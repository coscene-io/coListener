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

#include <rclcpp/rclcpp.hpp>
#include "listener.hpp"
#include "colistener.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("colistener"),
                "coListener - ROS2, version: %s, git hash: %s",
                colistener::VERSION, colistener::GIT_HASH);

    const auto node = std::make_shared<ros2_listener::Listener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
