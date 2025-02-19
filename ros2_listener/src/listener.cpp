#include <sstream>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <stack>
#include <utility>

#include <rclcpp/serialization.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <typesupport_helpers.hpp>

#include "listener.hpp"
#include "utils/logger.hpp"

namespace ros2_listener {
Listener::Listener() : Node("colistener") {
    this->declare_parameter("log_directory", "/tmp/colistener/logs/");
    const std::string log_directory = this->get_parameter("log_directory").as_string();
    colistener::Logger::getInstance().set_log_dir(log_directory);

    COLOG_INFO("coListener - ROS2, version: %s, git hash: %s", colistener::VERSION, colistener::GIT_HASH);
    COLOG_INFO("log directory: %s", log_directory.c_str());

    this->declare_parameter("action_type", "example");
    const std::string action_type = this->get_parameter("action_type").as_string();
    action_ = colistener::Action::create(action_type);
    COLOG_INFO("action_type: %s", action_type.c_str());

    this->declare_parameter("persistence_file_path", "/tmp/colistener/persistence/ros2.db");
    const std::string persistence_file = this->get_parameter("persistence_file_path").as_string();
    this->declare_parameter("persistence_expire_secs", 3600);
    const int64_t persistence_secs = this->get_parameter("persistence_expire_secs").as_int();
    database_manager_.init(persistence_file, persistence_secs);
    COLOG_INFO("persistence_file: %s, expire_secs: %d", persistence_file.c_str(), persistence_secs);

    this->declare_parameter("subscribe_topics", std::vector<std::string>{"/yolo/detections", "/error_event"});
    const std::vector<std::string> topics = this->get_parameter("subscribe_topics").as_string_array();
    pending_topics_ = topics;
    COLOG_INFO("Subscribing to topics: %s", colistener::vector_to_string<std::string>(topics).c_str());

    check_and_subscribe_topics();
    retry_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                           [this] {
                                               check_and_subscribe_topics();
                                           });

    send_timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                          [this] {
                                              batch_send_msgs_callback();
                                          });
}

void Listener::check_and_subscribe_topics() {
    if (pending_topics_.empty()) {
        retry_timer_->cancel();
        return;
    }

    auto topic_names_and_types = this->get_topic_names_and_types();
    RCLCPP_INFO(this->get_logger(), "current topic count: %d", topic_names_and_types.size());

    for (auto it = pending_topics_.begin(); it != pending_topics_.end();) {
        const auto& topic = *it;
        auto topic_it = topic_names_and_types.find(topic);

        if (topic_it != topic_names_and_types.end() && !topic_it->second.empty()) {
            const auto& datatypes = topic_it->second;
            bool subscription_success = false;

            try {
                for (const auto& datatype : datatypes) {
#ifdef ROS2_VERSION_HUMBLE
                    rclcpp::SubscriptionEventCallbacks event_callbacks;
                    event_callbacks.incompatible_qos_callback =
                        [this, topic, datatype](const rclcpp::QOSRequestedIncompatibleQoSInfo &) {
                            COLOG_ERROR("Incompatible subscriber QoS settings for topic \"%s\" (%s)",
                                topic.c_str(), datatype.c_str());
                        };

                    rclcpp::SubscriptionOptions subscription_options;
                    subscription_options.event_callbacks = event_callbacks;

                    auto subscriber = this->create_generic_subscription(
                        topic, datatype, get_qos_from_topic(topic),
                        [this, topic, datatype](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                            this->callback(msg, topic, datatype);
                        },
                        subscription_options);
                    subscribers_.push_back(subscriber);
                    subscription_success = true;
#endif

#ifdef ROS2_VERSION_FOXY
                    auto subscriber = create_generic_subscription(
                        this->get_node_topics_interface(),
                        topic, datatype, get_qos_from_topic(topic),
                        [this, topic, datatype](const std::shared_ptr<rclcpp::SerializedMessage>& msg) {
                            this->callback(msg, topic, datatype);
                        });
                    subscribers_.push_back(subscriber);
                    subscription_success = true;
#endif
                }
            }
            catch (const std::exception& e) {
                COLOG_ERROR("Failed to subscribe to topic '%s': %s", topic.c_str(), e.what());
            }

            if (subscription_success) {
                COLOG_INFO("Successfully subscribed to topic: %s", topic.c_str());
                it = pending_topics_.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void Listener::callback(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                        const std::string& topic,
                        const std::string& datatype) {
    try {
        const auto library = get_typesupport_library(
            datatype, "rosidl_typesupport_introspection_cpp");

        const auto type_support = get_typesupport_handle(
            datatype,
            "rosidl_typesupport_introspection_cpp",
            library);

        const auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
            type_support->data);

        const auto fields = build_message_fields(members);
        nlohmann::json json_msg;
        size_t offset = 0;
        size_t align_pos = 0;
        const auto msg_struct = msg->get_rcl_serialized_message();

        std::vector<uint8_t> src_msg;
        src_msg.resize(msg_struct.buffer_length);
        for (size_t i = 0; i < msg_struct.buffer_length; ++i) {
            src_msg[i] = msg_struct.buffer[i];
        }
        const auto debug_str = colistener::uint8_vector_to_string(src_msg);
        RCLCPP_INFO(this->get_logger(), "message data: %s", debug_str.c_str());

        // skip fucking first 4 bytes, it's version? header? or something else?, who knows~!
        deserialize_to_json(msg_struct.buffer, offset, align_pos, fields, json_msg);

        database_manager_.insert_message(colistener::MessageCache{
            topic,
            json_msg.dump(),
            datatype,
            this->now().seconds()
        });
    }
    catch (const std::exception& e) {
        COLOG_ERROR("Error processing message on topic '%s': %s", topic.c_str(), e.what());
    }
}

void Listener::batch_send_msgs_callback() {
    const auto msgs = database_manager_.get_all_messages();
    if (msgs.empty()) {
        return;
    }

    if (action_->execute(msgs)) {
        database_manager_.remove_messages(msgs);
    } else {
        COLOG_ERROR("Failed to send messages");
    }
}

rclcpp::QoS Listener::get_qos_from_topic(const std::string& topic) const {
    size_t depth = 0;
    size_t reliability_reliable_endpoints_count = 0;
    size_t durability_transient_local_endpoints_count = 0;

    const auto publisher_info = this->get_publishers_info_by_topic(topic);
    for (const auto& publisher : publisher_info) {
        const auto& qos = publisher.qos_profile();
        if (qos.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE) {
            ++reliability_reliable_endpoints_count;
        }
        if (qos.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
            ++durability_transient_local_endpoints_count;
        }
        const size_t publisher_history_depth = std::max(1ul, qos.get_rmw_qos_profile().depth);
        depth = depth + publisher_history_depth;
    }

    depth = std::max(depth, static_cast<size_t>(DEFAULT_MIN_QOS_DEPTH));
    if (depth > DEFAULT_MAX_QOS_DEPTH) {
        COLOG_WARN("Limiting history depth for topic '%s' to %zu (was %zu). You may want to increase "
                   "the max_qos_depth parameter value.",
                   topic.c_str(), DEFAULT_MAX_QOS_DEPTH, depth);
        depth = DEFAULT_MAX_QOS_DEPTH;
    }

    rclcpp::QoS qos{rclcpp::KeepLast(depth)};

    // If all endpoints are reliable, ask for reliable
    if (reliability_reliable_endpoints_count == publisher_info.size()) {
        qos.reliable();
    } else {
        if (reliability_reliable_endpoints_count > 0) {
            COLOG_WARN("Some, but not all, publishers on topic '%s' are offering QoSReliabilityPolicy.RELIABLE. "
                       "Falling back to QoSReliabilityPolicy.BEST_EFFORT as it will connect to all publishers",
                       topic.c_str());
        }
        qos.best_effort();
    }

    // If all endpoints are transient_local, ask for transient_local
    if (durability_transient_local_endpoints_count == publisher_info.size()) {
        qos.transient_local();
    } else {
        if (durability_transient_local_endpoints_count > 0) {
            COLOG_WARN("Some, but not all, publishers on topic '%s' are offering "
                       "QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to "
                       "QoSDurabilityPolicy.VOLATILE as it will connect to all publishers",
                       topic.c_str());
        }
        qos.durability_volatile();
    }
    return qos;
}

std::shared_ptr<GenericSubscription> Listener::create_generic_subscription(
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
    const std::string& topic,
    const std::string& type,
    const rclcpp::QoS& qos,
    const std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)>& callback) {
    const auto library_generic_subscriber = get_typesupport_library(
        type, "rosidl_typesupport_cpp");
    const auto type_support = get_typesupport_handle(
        type, "rosidl_typesupport_cpp", library_generic_subscriber);
    auto subscription = std::shared_ptr<GenericSubscription>();

    try {
        subscription = std::make_shared<GenericSubscription>(
            topics_interface->get_node_base_interface(),
            *type_support,
            topic,
            type,
            qos,
            callback);

        topics_interface->add_subscription(subscription, nullptr);
    }
    catch (const std::runtime_error& ex) {
        throw std::runtime_error("Error subscribing to topic '" + topic + "'. Error: " + ex.what());
    }
    return subscription;
}

void Listener::deserialize_to_json(const uint8_t* buffer, size_t& offset, size_t& align_pos,
                                   const std::vector<colistener::MessageField>& fields,
                                   nlohmann::json& json_msg) {
    // 检查大小端标记（第一个字节）
    bool is_little_endian = (buffer[0] == 0x01);
    COLOG_INFO("Message endianness: %s", is_little_endian ? "little-endian" : "big-endian");

    // 跳过4字节头部
    offset = 4;
    align_pos = 4;

    struct StackItem {
        const uint8_t* buffer;
        const std::vector<colistener::MessageField>* fields;
        nlohmann::json* json_msg;
        size_t field_index;
        uint32_t array_index;
        uint32_t array_size;
        bool is_array_processing;
    };

    std::stack<StackItem> stack;
    stack.push({buffer, &fields, &json_msg, 0, 0, 0, false});

    while (!stack.empty()) {
        auto& current = stack.top();

        if (current.field_index >= current.fields->size()) {
            stack.pop();
            continue;
        }

        const auto& field = (*current.fields)[current.field_index];

        if (field.is_array) {
            if (!current.is_array_processing) {
                uint32_t array_size = 0;
                RCLCPP_INFO(this->get_logger(), "array: read array length from offset: %u", offset);

                offset = align_pos + ((offset - align_pos + 3) & ~3);
                align_pos = offset;
                RCLCPP_INFO(this->get_logger(), "array: read array length from offset: %u", offset);
                std::memcpy(&array_size, buffer + offset, sizeof(uint32_t));
                offset += sizeof(uint32_t);

                (*current.json_msg)[field.name] = nlohmann::json::array();
                current.array_size = array_size;
                current.array_index = 0;
                current.is_array_processing = true;
            }

            if (current.array_index >= current.array_size) {
                current.is_array_processing = false;
                current.field_index++;
                continue;
            }

            if (field.is_builtin) {
                nlohmann::json element;
                deserialize_builtin_type(buffer, offset, align_pos, field.type, element, is_little_endian);
                (*current.json_msg)[field.name].push_back(element);
            } else {
                nlohmann::json sub_msg;
                stack.push({buffer, &field.sub_fields, &sub_msg, 0, 0, 0, false});
                (*current.json_msg)[field.name].push_back(sub_msg);
            }
            current.array_index++;
        } else {
            if (field.is_builtin) {
                deserialize_builtin_type(buffer, offset, align_pos, field.type,
                                         (*current.json_msg)[field.name],
                                         is_little_endian);
                current.field_index++;
            } else {
                nlohmann::json sub_msg;
                (*current.json_msg)[field.name] = sub_msg;
                stack.push({
                    buffer, &field.sub_fields,
                    &(*current.json_msg)[field.name], 0, 0, 0, false
                });
                current.field_index++;
            }
        }
    }
}

void Listener::deserialize_builtin_type(const uint8_t* buffer, size_t& offset, size_t& align_pos,
                                        colistener::RosDataType type,
                                        nlohmann::json& value,
                                        bool is_little_endian) {
    switch (type) {
        case colistener::RosDataType::RosDataTypeFloat: {
            float num_value;
            RCLCPP_INFO(this->get_logger(), "float: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset = align_pos + ((offset - align_pos + 3) & ~3);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "float: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(float));
            offset += sizeof(float);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "float value: %lf", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeDouble: {
            double num_value;
            RCLCPP_INFO(this->get_logger(), "double: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset = align_pos + ((offset - align_pos + 7) & ~7);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "double: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(double));
            offset += sizeof(double);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "double value: %lf", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeBool: {
            uint8_t bool_value;
            std::memcpy(&bool_value, buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            align_pos = offset;
            value = static_cast<bool>(bool_value);
            RCLCPP_INFO(this->get_logger(), "bool value: %d", bool_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeOctet: {
            uint8_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            align_pos = offset;
            value = static_cast<unsigned int>(num_value);
            RCLCPP_INFO(this->get_logger(), "octet value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeUint8: {
            uint8_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(uint8_t));
            offset += sizeof(uint8_t);
            align_pos = offset;
            value = static_cast<unsigned int>(num_value);
            RCLCPP_INFO(this->get_logger(), "uint8 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeInt8: {
            int8_t num_value;
            std::memcpy(&num_value, buffer + offset, sizeof(int8_t));
            offset += sizeof(int8_t);
            align_pos = offset;
            value = static_cast<int>(num_value);
            RCLCPP_INFO(this->get_logger(), "int8 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeUint16: {
            uint16_t num_value;
            RCLCPP_INFO(this->get_logger(), "uint16: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 1) & ~1);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "uint16: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(uint16_t));
            offset += sizeof(uint16_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "uint16 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeInt16: {
            int16_t num_value;
            RCLCPP_INFO(this->get_logger(), "int16: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 1) & ~1);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "int16: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(int16_t));
            offset += sizeof(int16_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "int16 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeUint32: {
            uint32_t num_value;
            RCLCPP_INFO(this->get_logger(), "uint32: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 3) & ~3);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "uint32: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "uint32 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeInt32: {
            int32_t num_value;
            RCLCPP_INFO(this->get_logger(), "int32: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 3) & ~3);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "int32: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(int32_t));
            offset += sizeof(int32_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "int32 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeUint64: {
            uint64_t num_value;
            RCLCPP_INFO(this->get_logger(), "uint64: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 7) & ~7);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "uint64: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(uint64_t));
            offset += sizeof(uint64_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "uint64 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeInt64: {
            int64_t num_value;
            RCLCPP_INFO(this->get_logger(), "int64: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 7) & ~7);
            align_pos = offset;
            RCLCPP_INFO(this->get_logger(), "int64: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&num_value, buffer + offset, sizeof(int64_t));
            offset += sizeof(int64_t);
            value = num_value;
            RCLCPP_INFO(this->get_logger(), "int64 value: %d", num_value);
            break;
        }
        case colistener::RosDataType::RosDataTypeString: {
            uint32_t str_len = 0;
            RCLCPP_INFO(this->get_logger(), "string: read data from offset: %u, alignment pos: %u", offset, align_pos);
            offset =  align_pos + ((offset - align_pos + 3) & ~3);
            RCLCPP_INFO(this->get_logger(), "string: read data after alignment offset: %u, alignment pos: %u", offset, align_pos);
            std::memcpy(&str_len, buffer + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);
            align_pos = offset;

            if (str_len > 0) {
                std::string str(reinterpret_cast<const char*>(buffer + offset), str_len - 1); // remove null
                offset += str_len;
                value = str;
                RCLCPP_INFO(this->get_logger(), "string value: \"%s\"", str.c_str());
            } else {
                value = "";
                RCLCPP_INFO(this->get_logger(), "string value: \"\"");
            }
            break;
        }
        default:
            if (is_little_endian) {
                throw std::runtime_error("Unsupported builtin type.");
            }
    }
}

colistener::RosDataType Listener::convert_to_rostype(const uint8_t ros_type) {
    colistener::RosDataType type;
    switch (ros_type) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
            type = colistener::RosDataType::RosDataTypeBool;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
            type = colistener::RosDataType::RosDataTypeOctet;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            type = colistener::RosDataType::RosDataTypeUint8;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            type = colistener::RosDataType::RosDataTypeInt8;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            type = colistener::RosDataType::RosDataTypeUint16;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            type = colistener::RosDataType::RosDataTypeInt16;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            type = colistener::RosDataType::RosDataTypeUint32;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            type = colistener::RosDataType::RosDataTypeInt32;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            type = colistener::RosDataType::RosDataTypeUint64;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            type = colistener::RosDataType::RosDataTypeInt64;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32:
            type = colistener::RosDataType::RosDataTypeFloat;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64:
            type = colistener::RosDataType::RosDataTypeDouble;
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
            type = colistener::RosDataType::RosDataTypeString;
            break;
        default:
            type = colistener::RosDataType::RosDataTypeUnknown;
    }
    return type;
}

std::vector<colistener::MessageField> Listener::build_message_fields(
    const rosidl_typesupport_introspection_cpp::MessageMembers* members) {
    struct StackItem {
        const rosidl_typesupport_introspection_cpp::MessageMembers* members;
        std::vector<colistener::MessageField>* fields;
        size_t field_index;
    };

    std::stack<StackItem> stack;
    std::vector<colistener::MessageField> result;
    stack.push({members, &result, 0});

    while (!stack.empty()) {
        auto& current = stack.top();

        if (current.field_index >= current.members->member_count_) {
            stack.pop();
            continue;
        }

        const auto& member = current.members->members_[current.field_index];
        colistener::MessageField field;
        field.name = member.name_;
        field.is_array = member.is_array_;

        if (member.type_id_ < rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            field.is_builtin = true;
            field.type = convert_to_rostype(member.type_id_);
            current.fields->push_back(std::move(field));
            current.field_index++;
        } else {
            field.is_builtin = false;
            if (!member.members_ || !member.members_->data) {
                throw std::runtime_error("Invalid message members for field: " + std::string(member.name_));
            }

            const auto* sub_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(
                member.members_->data);

            if (!sub_members) {
                throw std::runtime_error("Failed to get sub-message members for field: " + std::string(member.name_));
            }

            field.package_name = sub_members->message_namespace_;
            field.type = colistener::RosDataType::RosDataTypeMessage;
            field.sub_fields = std::vector<colistener::MessageField>();

            current.fields->push_back(std::move(field));

            stack.push({sub_members, &current.fields->back().sub_fields, 0});
            current.field_index++;
        }
    }

    return result;
}
} // namespace ros2_listener
