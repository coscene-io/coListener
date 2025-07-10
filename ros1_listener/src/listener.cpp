// Copyright 2025 coScene
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>
#include <cstring>
#include <map>
#include <stdexcept>
#include <fstream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "listener.hpp"
#include "utils/logger.hpp"
#include "utils/set_utils.hpp"

namespace ros1_listener {
const std::set<std::string> Listener::builtin_types_ = {
    "bool", "int8", "uint8", "int16", "uint16", "int32", "uint32",
    "int64", "uint64", "float32", "float64", "string", "time", "duration"
};

Listener::Listener() : nh_("~") {
    endpoint_ = std::string(colistener::DEFAULT_URL) + ":" +
        std::string(colistener::DEFAULT_PORT) + std::string(colistener::ACTIVE_TOPICS);
    headers_["Content-Type"] = "application/json";
    headers_["User-Agent"] = "coListener/1.0";

    std::string log_dir;
    if (!nh_.getParam("log_directory", log_dir)) {
        log_dir = "/tmp/colistener/log/";
    }
    colistener::Logger::getInstance().set_log_dir(log_dir);
#ifdef DEBUG_BUILD
    colistener::Logger::getInstance().set_log_level(colistener::LogLevel::DEBUG);
#else
    colistener::Logger::getInstance().set_log_level(colistener::LogLevel::INFO);
#endif

    COLOG_INFO("coListener - ROS1, version: %s, git hash: %s", colistener::VERSION, colistener::GIT_HASH);
    COLOG_INFO("log directory: %s", log_dir.c_str());

    std::string action_type;
    if (!nh_.getParam("action_type", action_type)) {
        action_type = "example";
        COLOG_WARN("No action type specified, using default: %s", action_type.c_str());
    }
    action_ = colistener::Action::create(action_type);
    COLOG_INFO("action type: %s", action_type.c_str());

    std::string db_path;
    int persistence_expire_interval_secs;
    if (!nh_.getParam("persistence_file_path", db_path)) {
        db_path = "/tmp/colistener/persistence/ros1.db";
        COLOG_WARN("No persistence file path specified, using default: %s", db_path.c_str());
    }
    if (!nh_.getParam("persistence_expire_secs", persistence_expire_interval_secs)) {
        persistence_expire_interval_secs = 3600;
        COLOG_WARN("No persistence expire interval specified, using default: %d", persistence_expire_interval_secs);
    }
    database_manager_.init(db_path, persistence_expire_interval_secs);
    COLOG_INFO("persistence_file: %s, expire_secs: %d", db_path.c_str(), persistence_expire_interval_secs);

    send_messages_timer_ = nh_.createTimer(ros::Duration(5.0),
        [this](const ros::TimerEvent& event) { this->sending_messages(event); });
    update_subscriptions_timer_ = nh_.createTimer(ros::Duration(3.0), 
        [this](const ros::TimerEvent& event) { this->update_subscriptions(event); });
}

Listener::~Listener() {
    send_messages_timer_.stop();
    update_subscriptions_timer_.stop();
}

void Listener::callback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg,
                        const std::string& topic) {
    nlohmann::json json_msg;
    const std::string& datatype = msg->getDataType();
    const std::string& definition = msg->getMessageDefinition();

    std::vector<colistener::MessageField> fields;
    {
        std::lock_guard<std::mutex> lock(message_definitions_mutex_);
        auto it = message_definitions_.find(datatype);
        if (it == message_definitions_.end()) {
            message_definitions_[datatype] = parse_message_definition(definition);
        }
        fields = message_definitions_[datatype];
    }

    const size_t len = msg->size();
    std::vector<uint8_t> buffer(len);
    ros::serialization::OStream stream(buffer.data(), len);
    msg->write(stream);

    size_t offset = 0;
    deserialize_to_json(buffer.data(), offset, message_definitions_[datatype], json_msg);
    const auto msg_json_str = json_msg.dump();
    database_manager_.insert_message(colistener::MessageCache{
        topic,
        msg_json_str,
        datatype,
        ros::Time::now().toSec()
    });
}

void Listener::update_subscriptions(const ros::TimerEvent&)
{
    std::set<std::string> topics;
    colistener::HttpResponse resp = curl_client_.get(endpoint_, headers_);
    if (resp.success) {
        try {
            nlohmann::json response_json = nlohmann::json::parse(resp.body);

            if (response_json.contains("topics") && response_json["topics"].is_array()) {
                const auto& active_topics = response_json["topics"];
                for (const auto& topic : active_topics) {
                    if (topic.is_string()) {
                        topics.emplace(topic.get<std::string>());
                    }
                }
                COLOG_DEBUG("Received %zu active topics from server", topics.size());
            }
        } catch (const nlohmann::json::parse_error& e) {
            COLOG_ERROR("Failed to parse JSON response: %s", e.what());
        }
    } else {
        COLOG_ERROR("GET request failed: %s", resp.error_message.c_str());
    }

    const auto diff = colistener::findSetsDifference(subscribe_topics_, topics);
    if (diff.isIdentical()) {
        COLOG_DEBUG("no topic was missing or added, skip.");
        return;
    }
    if (!diff.missing.empty()) {
        for (const auto& removed_topic : diff.missing) {
            subscriptions_[removed_topic].shutdown();
            subscriptions_.erase(removed_topic);
            subscribe_topics_.erase(removed_topic);
            COLOG_INFO("remove topic [%s] from subscriptions list.", removed_topic.c_str());
        }
    }

    if (!diff.added.empty()) {
        for (const auto& added_topic : diff.added) {
            ros::Subscriber sub = nh_.subscribe<topic_tools::ShapeShifter>(
                added_topic, 10,
                [this, added_topic](const topic_tools::ShapeShifter::ConstPtr& msg) {
                    this->callback(msg, added_topic);
                }
            );
            subscriptions_.emplace(added_topic, sub);
            subscribe_topics_.emplace(added_topic);
            COLOG_INFO("subscribe topic: %s, subscriptions length :%d", added_topic.c_str(), subscriptions_.size());
        }
    }
}

void Listener::sending_messages(const ros::TimerEvent&) {
    const auto msgs = database_manager_.get_all_messages();
    if (msgs.empty()) {
        return;
    }

    if (action_->execute(msgs)) {
        database_manager_.remove_messages(msgs);
    } else {
        COLOG_INFO("Failed to send messages");
    }
}

RosDataType Listener::convert_to_rostype(const std::string& type) {
    if (type == "string") {
        return RosDataType::RosDataTypeString;
    } else if (type == "bool") {
        return RosDataType::RosDataTypeBool;
    } else if (type == "int8") {
        return RosDataType::RosDataTypeInt8;
    } else if (type == "uint8") {
        return RosDataType::RosDataTypeUint8;
    } else if (type == "int16") {
        return RosDataType::RosDataTypeInt16;
    } else if (type == "uint16") {
        return RosDataType::RosDataTypeUint16;
    } else if (type == "int32") {
        return RosDataType::RosDataTypeInt32;
    } else if (type == "uint32") {
        return RosDataType::RosDataTypeUint32;
    } else if (type == "int64") {
        return RosDataType::RosDataTypeInt64;
    } else if (type == "uint64") {
        return RosDataType::RosDataTypeUint64;
    } else if (type == "float32") {
        return RosDataType::RosDataTypeFloat;
    } else if (type == "float64") {
        return RosDataType::RosDataTypeDouble;
    } else if (type == "duration" || type == "time") {
        return RosDataType::RosDataTypeTime;
    } else {
        return RosDataType::RosDataTypeMessage;
    }
}

std::vector<colistener::MessageField> Listener::parse_message_definition(const std::string& definition) {
    std::map<std::string, std::string> message_sections;
    std::string current_msg_type;
    std::string current_section;
    bool first_section = true;
    std::string main_section;

    std::istringstream full_iss(definition);
    std::string line;
    while (std::getline(full_iss, line)) {
        if (line.find("================================================================================") !=
            std::string::npos) {
            if (first_section) {
                main_section = current_section;
                first_section = false;
            } else if (!current_msg_type.empty()) {
                message_sections[current_msg_type] = current_section;
            }
            current_section.clear();
            continue;
        }

        if (line.find("MSG: ") == 0) {
            current_msg_type = line.substr(5);
            continue;
        }

        current_section += line + "\n";
    }
    if (!first_section && !current_msg_type.empty()) {
        message_sections[current_msg_type] = current_section;
    } else if (first_section) {
        main_section = current_section;
    }

    return parse_section(main_section, message_sections);
}

std::vector<colistener::MessageField> Listener::parse_section(const std::string& section,
                                                          const std::map<std::string, std::string>&
                                                          message_sections) {
    std::vector<colistener::MessageField> fields;
    std::istringstream iss(section);
    std::string line;

    while (std::getline(iss, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty()) {
            continue;
        }
        if (line.find('=') != std::string::npos) {
            continue;
        }

        std::istringstream field_iss(line);
        std::string type, name;

        field_iss >> type;
        std::getline(field_iss, name);
        name.erase(0, name.find_first_not_of(" \t"));

        size_t comment_pos = name.find('#');
        if (comment_pos != std::string::npos) {
            name = name.substr(0, comment_pos);
            name.erase(name.find_last_not_of(" \t") + 1);
        }

        if (name.back() == ';') {
            name.pop_back();
        }

        if (!type.empty() && !name.empty()) {
            colistener::MessageField field;
            field.name = name;
            field.is_array = (type.find("[]") != std::string::npos);
            if (field.is_array) {
                type = type.substr(0, type.length() - 2);
            }

            field.is_builtin = (builtin_types_.find(type) != builtin_types_.end());

            if (!field.is_builtin) {
                size_t slash_pos = type.find('/');
                field.type = RosDataType::RosDataTypeMessage;
                std::string message_type;
                if (slash_pos != std::string::npos) {
                    field.package_name = type.substr(0, slash_pos);
                    message_type = type.substr(slash_pos + 1);
                } else {
                    message_type = type;
                }

                std::string full_type = field.package_name.empty()
                                            ? message_type
                                            : field.package_name + "/" + message_type;

                for (const auto& msg_section : message_sections) {
                    if (msg_section.first.find(full_type) != std::string::npos) {
                        field.package_name = msg_section.first.substr(0, msg_section.first.find('/'));
                        field.sub_fields = parse_section(msg_section.second, message_sections);
                        break;
                    }
                }
            } else {
                field.type = convert_to_rostype(type);
            }

            fields.push_back(field);
        }
    }

    return fields;
}

void Listener::deserialize_to_json(const uint8_t* buffer, size_t& offset,
                                   const std::vector<colistener::MessageField>& fields,
                                   nlohmann::json& json_msg) {
    for (const auto& field : fields) {
        if (field.is_array) {
            uint32_t array_len;
            std::memcpy(&array_len, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            nlohmann::json array = nlohmann::json::array();
            for (uint32_t i = 0; i < array_len; ++i) {
                if (field.is_builtin) {
                    nlohmann::json element;
                    deserialize_builtin_type(buffer, offset, field.type, element);
                    array.push_back(element);
                } else {
                    nlohmann::json sub_msg;
                    deserialize_to_json(buffer, offset, field.sub_fields, sub_msg);
                    array.push_back(sub_msg);
                }
            }
            json_msg[field.name] = array;
        } else {
            if (field.is_builtin) {
                deserialize_builtin_type(buffer, offset, field.type, json_msg[field.name]);
            } else {
                nlohmann::json sub_msg;
                deserialize_to_json(buffer, offset, field.sub_fields, sub_msg);
                json_msg[field.name] = sub_msg;
            }
        }
    }
}

void Listener::deserialize_builtin_type(const uint8_t* buffer, size_t& offset,
                                        RosDataType type,
                                        nlohmann::json& value) {
    switch (type) {
        case RosDataType::RosDataTypeString: {
            uint32_t str_len;
            std::memcpy(&str_len, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            std::string str_value(reinterpret_cast<const char*>(buffer + offset), str_len);
            offset += str_len;
            value = str_value;
            break;
        }
        case RosDataType::RosDataTypeBool: {
            uint8_t bool_value;
            std::memcpy(&bool_value, buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            value = static_cast<bool>(bool_value);
            break;
        }
        case RosDataType::RosDataTypeInt8: {
            int8_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(int8_t));
            offset += sizeof(int8_t);
            value = static_cast<int>(num_value);
            break;
        }
        case RosDataType::RosDataTypeUint8: {
            uint8_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            value = static_cast<unsigned int>(num_value);
            break;
        }
        case RosDataType::RosDataTypeInt16: {
            int16_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(int16_t));
            offset += sizeof(int16_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeUint16: {
            uint16_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint16_t));
            offset += sizeof(uint16_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeInt32: {
            int32_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(int32_t));
            offset += sizeof(int32_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeUint32: {
            uint32_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeInt64: {
            int64_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(int64_t));
            offset += sizeof(int64_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeUint64: {
            uint64_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint64_t));
            offset += sizeof(uint64_t);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeFloat: {
            float num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(float));
            offset += sizeof(float);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeDouble: {
            double num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(double));
            offset += sizeof(double);
            value = num_value;
            break;
        }
        case RosDataType::RosDataTypeTime: {
            uint32_t secs, nsecs;
            std::memcpy(&secs, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            std::memcpy(&nsecs, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            nlohmann::json time_obj;
            time_obj["secs"] = secs;
            time_obj["nsecs"] = nsecs;
            value = time_obj;
            break;
        }
        default:
            throw std::invalid_argument("Unknown type");
    }
}
}  // namespace ros1_listener
