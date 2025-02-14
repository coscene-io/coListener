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

#include <ros/ros.h>
#include <string>
#include <topic_tools/shape_shifter.h>
#include "listener.hpp"
#include "colistener.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "colistener");

    ROS_INFO("coListener - ROS1, version: %s, git hash: %s", colistener::VERSION, colistener::GIT_HASH);
    ros1_listener::Listener node;
    ros::spin();
    return 0;
}
