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

#ifndef ROS2_LISTENER_GENERIC_SUBSCRIPTION_HPP_
#define ROS2_LISTENER_GENERIC_SUBSCRIPTION_HPP_


#include <memory>
#include <string>
#include "rclcpp/subscription.hpp"

namespace ros2_listener {
class GenericSubscription : public rclcpp::SubscriptionBase {
public:
    using SharedPtr = std::shared_ptr<GenericSubscription>;

    GenericSubscription(const GenericSubscription&) = delete;
    GenericSubscription& operator=(const GenericSubscription&) = delete;

    GenericSubscription(rclcpp::node_interfaces::NodeBaseInterface* node_base, const rosidl_message_type_support_t& ts,
                        std::string topic_name, std::string topic_type, const rclcpp::QoS& qos,
                        std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback);
    std::shared_ptr<void> create_message() override;
    std::shared_ptr<rclcpp::SerializedMessage> create_serialized_message() override;

    void handle_message(
        std::shared_ptr<void>& message, const rclcpp::MessageInfo& message_info) override;

    void handle_loaned_message(
        void* loaned_message, const rclcpp::MessageInfo& message_info) override;

#ifdef ROS2_VERSION_HUMBLE
	void handle_serialized_message(
		const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    	const rclcpp::MessageInfo & message_info) override;
#endif

    void return_message(std::shared_ptr<void>& message) override;

    void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage>& message) override;

    const rclcpp::QoS& qos_profile() const;

private:
    static std::shared_ptr<rclcpp::SerializedMessage> borrow_serialized_message(size_t capacity);

    rcutils_allocator_t _default_allocator;
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> _callback;
    const rclcpp::QoS _qos;
    int64_t _last_frame_timestamp;
    std::string _message_type;
    std::string _topic_name;
};
}
#endif  // ROS2_LISTENER_GENERIC_SUBSCRIPTION_HPP_
