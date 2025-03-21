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

#include "generic_subscription.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/subscription.hpp"

namespace {
rcl_subscription_options_t get_subscription_options(const rclcpp::QoS& qos) {
    auto options = rcl_subscription_get_default_options();
    options.qos = qos.get_rmw_qos_profile();
    return options;
}
}

namespace ros2_listener {
GenericSubscription::GenericSubscription(
    rclcpp::node_interfaces::NodeBaseInterface* node_base,
    const rosidl_message_type_support_t& ts,
    std::string topic_name,
    std::string topic_type,
    const rclcpp::QoS& qos,
    std::function<void(std::shared_ptr<rclcpp::SerializedMessage>)> callback)
    : SubscriptionBase(node_base, ts, topic_name, get_subscription_options(qos), true),
      _default_allocator(rcutils_get_default_allocator()),
      _callback(std::move(callback)),
      _qos(qos),
      _last_frame_timestamp(0),
      _message_type(std::move(topic_type)),
      _topic_name(std::move(topic_name)) {
}

std::shared_ptr<void> GenericSubscription::create_message() {
    return create_serialized_message();
}

std::shared_ptr<rclcpp::SerializedMessage> GenericSubscription::create_serialized_message() {
    return borrow_serialized_message(0);
}

void GenericSubscription::handle_message(
    std::shared_ptr<void>& message, const rclcpp::MessageInfo& message_info) {
    (void) message_info;
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    _callback(typed_message);
}

void GenericSubscription::handle_loaned_message(
    void* message, const rclcpp::MessageInfo& message_info) {
    (void)message;
    (void)message_info;
}

#ifdef ROS2_VERSION_HUMBLE
void GenericSubscription::handle_serialized_message(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_message,
    const rclcpp::MessageInfo & message_info){
    (void)message_info;
    _callback(serialized_message);
}
#endif

void GenericSubscription::return_message(std::shared_ptr<void>& message) {
    auto typed_message = std::static_pointer_cast<rclcpp::SerializedMessage>(message);
    return_serialized_message(typed_message);
}

void GenericSubscription::return_serialized_message(
    std::shared_ptr<rclcpp::SerializedMessage>& message) {
    message.reset();
}

const rclcpp::QoS& GenericSubscription::qos_profile() const {
    return _qos;
}

std::shared_ptr<rclcpp::SerializedMessage>
GenericSubscription::borrow_serialized_message(size_t capacity) {
    return std::make_shared<rclcpp::SerializedMessage>(capacity);
}
}  // namespace ros2_listener
