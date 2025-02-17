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

#ifndef ROS2_LISTENER_LISTENER_HPP
#define ROS2_LISTENER_LISTENER_HPP

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "actions/action.hpp"
#include "persistence/database_manager.hpp"
#include "utils/logger.hpp"
#include "colistener.hpp"
#include "generic_subscription.hpp"
#include <mutex>
#include <thread>
#include <curl/curl.h>

namespace ros2_listener {
constexpr int64_t DEFAULT_MIN_QOS_DEPTH = 1;
constexpr int64_t DEFAULT_MAX_QOS_DEPTH = 25;

class Listener : public rclcpp::Node {
public:
    Listener();
    ~Listener() override = default;

private:
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
    std::map<std::string, std::vector<colistener::MessageField>> message_definitions_;
    std::vector<std::string> pending_topics_;
    rclcpp::TimerBase::SharedPtr retry_timer_;

    rclcpp::TimerBase::SharedPtr send_timer_;
    std::shared_ptr<colistener::Action> action_;
    colistener::DatabaseManager database_manager_;

    void check_and_subscribe_topics();

    void batch_send_msgs_callback();

    void callback(const std::shared_ptr<rclcpp::SerializedMessage>& msg,
                  const std::string& topic,
                  const std::string& datatype);

    static std::vector<colistener::MessageField> build_message_fields(
        const rosidl_typesupport_introspection_cpp::MessageMembers* members);

    static std::shared_ptr<GenericSubscription> create_generic_subscription(
        const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr& topics_interface,
        const std::string& topic,
        const std::string& type,
        const rclcpp::QoS& qos,
        const std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)>& callback);

    static void deserialize_to_json(const uint8_t* buffer, size_t& offset,
                                    const std::vector<colistener::MessageField>& fields,
                                    nlohmann::json& json_msg);

    static void deserialize_builtin_type(const uint8_t* buffer, size_t& offset,
                                         colistener::RosDataType type,
                                         nlohmann::json& value);

    static colistener::RosDataType convert_to_rostype(uint8_t ros_type);

    rclcpp::QoS get_qos_from_topic(const std::string& topic) const;
};
} // namespace ros2_listener

#endif // ROS2_LISTENER_LISTENER_HPP
