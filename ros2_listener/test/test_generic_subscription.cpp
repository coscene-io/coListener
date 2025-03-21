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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "create_generic_subscription.hpp"

using namespace std::chrono_literals;

class TestGenericSubscription : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_generic_subscription");
    string_publisher_ = node_->create_publisher<std_msgs::msg::String>("test_string_topic", 10);
    int32_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("test_int32_topic", 10);
  }

  void TearDown() override {
    string_publisher_.reset();
    int32_publisher_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int32_publisher_;
  std::vector<std::string> received_string_messages_;
  std::vector<int32_t> received_int32_messages_;
};

TEST_F(TestGenericSubscription, ReceivesStringMessages) {
  auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
    auto deser_msg = std::make_shared<std_msgs::msg::String>();
    rclcpp::Serialization<std_msgs::msg::String> serialization;
    serialization.deserialize_message(serialized_msg.get(), deser_msg.get());
    received_string_messages_.push_back(deser_msg->data);
  };

  auto subscription = ros2_listener::create_generic_subscription(
    node_->get_node_topics_interface(),
    "test_string_topic",
    "std_msgs/msg/String",
    rclcpp::QoS(10),
    callback
  );

  auto publish_message = [this](const std::string& msg_text) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = msg_text;
    string_publisher_->publish(std::move(msg));

    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    rclcpp::spin_some(node_);
  };

  std::vector<std::string> test_messages = {
    "Hello, World!",
    "Testing ROS2 generic subscription",
    "Final test message"
  };

  for (const auto& msg : test_messages) {
    publish_message(msg);
  }

  ASSERT_EQ(received_string_messages_.size(), test_messages.size())
    << "Expected to receive " << test_messages.size() << " messages, got "
    << received_string_messages_.size();

  for (size_t i = 0; i < test_messages.size(); ++i) {
    EXPECT_EQ(received_string_messages_[i], test_messages[i])
      << "Message at index " << i << " does not match expected value";
  }
}

TEST_F(TestGenericSubscription, ReceivesInt32Messages) {
  auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
    auto deser_msg = std::make_shared<std_msgs::msg::Int32>();
    rclcpp::Serialization<std_msgs::msg::Int32> serialization;
    serialization.deserialize_message(serialized_msg.get(), deser_msg.get());
    received_int32_messages_.push_back(deser_msg->data);
  };

  auto subscription = ros2_listener::create_generic_subscription(
    node_->get_node_topics_interface(),
    "test_int32_topic",
    "std_msgs/msg/Int32",
    rclcpp::QoS(10),
    callback
  );

  auto publish_message = [this](int32_t value) {
    auto msg = std::make_unique<std_msgs::msg::Int32>();
    msg->data = value;
    int32_publisher_->publish(std::move(msg));

    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(100ms);
    rclcpp::spin_some(node_);
  };

  std::vector<int32_t> test_values = {42, 100, -15, 0, 9999};

  for (const auto& value : test_values) {
    publish_message(value);
  }

  ASSERT_EQ(received_int32_messages_.size(), test_values.size())
    << "Expected to receive " << test_values.size() << " messages, got "
    << received_int32_messages_.size();

  for (size_t i = 0; i < test_values.size(); ++i) {
    EXPECT_EQ(received_int32_messages_[i], test_values[i])
      << "Message at index " << i << " does not match expected value";
  }
}

TEST_F(TestGenericSubscription, HandlesInvalidTopicType) {
  EXPECT_THROW({
    auto subscription = ros2_listener::create_generic_subscription(
      node_->get_node_topics_interface(),
      "test_topic",
      "invalid_package/msg/InvalidType",
      rclcpp::QoS(10),
      [](std::shared_ptr<rclcpp::SerializedMessage>) {}
    );
  }, std::runtime_error);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
