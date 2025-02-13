#include <sstream>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <stack>
#include <utility>

#include "listener.hpp"

namespace ros1_listener {


const std::set<std::string> Listener::builtin_types_ = {
    "bool", "int8", "uint8", "int16", "uint16", "int32", "uint32",
    "int64", "uint64", "float32", "float64", "string"
};

Listener::Listener() : curl_(nullptr), headers_(nullptr)  {
    ros::NodeHandle private_nh("~");

    curl_ = curl_easy_init();
    if (!curl_) {
        throw std::runtime_error("Failed to initialize CURL");
    }
    headers_ = curl_slist_append(nullptr, "Content-Type: application/json");

    std::vector<std::string> topics;
    if (!private_nh.getParam("subscribe_topics", topics)) {
        topics = {"/error_code", "/error_event"};
        ROS_WARN("No topics specified, using default topic: %s", vector_to_string<std::string>(topics).c_str());
    }
    ROS_INFO("Subscribing to topics: %s", vector_to_string<std::string>(topics).c_str());

    for (const auto& topic : topics) {
        ros::Subscriber sub = private_nh.subscribe<topic_tools::ShapeShifter>(
            topic, 10,
            [this, topic](const topic_tools::ShapeShifter::ConstPtr& msg) {
                this->callback(msg, topic);
            }
        );
        subscribers_.push_back(sub);
    }
    timer_thread_ = std::thread(&Listener::timer_callback, this);
}

Listener::~Listener() {
    running_ = false;
    if (timer_thread_.joinable()) {
        timer_thread_.join();
    }
    if (headers_) {
        curl_slist_free_all(headers_);
    }
    if (curl_) {
        curl_easy_cleanup(curl_);
    }
}

void Listener::callback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg,
                        const std::string& topic) {
    nlohmann::json json_msg;
    const std::string& datatype = msg->getDataType();
    const std::string& definition = msg->getMessageDefinition();

    auto it = message_definitions_.find(datatype);
    if (it == message_definitions_.end()) {
        message_definitions_[datatype] = parse_message_definition(definition);
    }

    const size_t len = msg->size();
    std::vector<uint8_t> buffer(len);
    ros::serialization::OStream stream(buffer.data(), len);
    msg->write(stream);

    size_t offset = 0;
    deserialize_to_json(buffer.data(), offset, message_definitions_[datatype], json_msg);
    ROS_INFO("Received message: %s", json_msg.dump().c_str());

    {
        colistener::MessageCache cache_item{
            topic,
            json_msg,
            datatype,
            ros::Time::now().toSec()
        };
        std::lock_guard<std::mutex> lock(cache_mutex_);
        message_cache_.push_back(std::move(cache_item));
    }
}

void Listener::timer_callback() {
    while (running_) {
        send_cached_messages();
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void Listener::send_cached_messages() {
    std::vector<colistener::MessageCache> messages_to_send;
    {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (message_cache_.empty()) {
            return;
        }
        messages_to_send.swap(message_cache_);
    }

    ROS_INFO("Preparing to send %zu cached messages", messages_to_send.size());

    nlohmann::json json_array = nlohmann::json::array();
    for (const auto& cache_item : messages_to_send) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        item["msg"] = cache_item.msg;
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        json_array.push_back(item);
    }

    try {
        if (!curl_) {
            ROS_WARN("CURL not initialized, cached messages will be dropped");
            return;
        }

        const std::string json_str = json_array.dump();
        ROS_DEBUG("Sending JSON payload (size: %zu bytes)", json_str.length());
        
        const std::string url = std::string(colistener::DEFAULT_URL) + 
                               std::string(":") + 
                               std::string(colistener::DEFAULT_PORT) + 
                               std::string(colistener::DEFAULT_ROUTE);
        
        curl_easy_setopt(curl_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers_);
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_str.c_str());
        curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_str.length());
        
        const CURLcode res = curl_easy_perform(curl_);
        if (res != CURLE_OK) {
            ROS_ERROR("Failed to send messages: %s", curl_easy_strerror(res));
        } else {
            long http_code = 0;
            curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &http_code);
            
            if (http_code >= 200 && http_code < 300) {
                ROS_INFO("Successfully sent %zu messages, HTTP status: %ld", 
                        messages_to_send.size(), http_code);
            } else {
                ROS_WARN("Server returned unexpected status code: %ld", http_code);
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while sending messages: %s", e.what());
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
    } else {
        return RosDataType::RosDataTypeMessage;
    }
}

// TODO(fei): DCL56-CPP remove recursion, just like ros2 listener did.
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
        if (line.find("================================================================================") !=
            std::string::npos) {
            break;
        }

        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (line.empty()) {
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

std::vector<colistener::MessageField> Listener::parse_message_definition(const std::string& definition) {
    std::map<std::string, std::string> message_sections;
    std::string current_msg_type;
    std::string current_section;

    std::istringstream full_iss(definition);
    std::string line;
    while (std::getline(full_iss, line)) {
        if (line.find("================================================================================") !=
            std::string::npos) {
            if (!current_msg_type.empty()) {
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
    if (!current_msg_type.empty()) {
        message_sections[current_msg_type] = current_section;
    }

    return parse_section(definition, message_sections);
}

// TODO(fei): DCL56-CPP remove recursion, just list ros2 listener did.
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
        default:
            throw std::invalid_argument("Unknown type");
    }
}
}
