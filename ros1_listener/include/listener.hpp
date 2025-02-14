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

#ifndef ROS1_LISTENER_LISTENER_H
#define ROS1_LISTENER_LISTENER_H

#include <vector>
#include <string>
#include <map>
#include <set>
#include <mutex>
#include <thread>

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "colistener.hpp"
#include "actions/action.hpp"
#include "persistence/database_manager.hpp"
#include "utils/logger.hpp"

namespace ros1_listener {

using RosDataType = colistener::RosDataType;

class Listener {
public:
    Listener();
    ~Listener();

private:
    ros::NodeHandle nh_;

    std::shared_ptr<colistener::Action> action_;
    std::vector<ros::Subscriber> subscribers_;
    std::map<std::string, std::vector<colistener::MessageField>> message_definitions_;
    static const std::set<std::string> builtin_types_;
    colistener::DatabaseManager database_manager_;

    std::mutex cache_mutex_;
    std::thread timer_thread_;
    bool running_ = true;

    CURL* curl_;
    struct curl_slist* headers_;

    void timer_callback();
    void send_cached_messages();

    void callback(const boost::shared_ptr<const topic_tools::ShapeShifter>& msg,
                  const std::string& topic);

    std::vector<colistener::MessageField> parse_section(const std::string& section,
                                                        const std::map<std::string, std::string>& message_sections);
    std::vector<colistener::MessageField> parse_message_definition(const std::string& definition);

    static void deserialize_to_json(const uint8_t* buffer, size_t& offset,
                                    const std::vector<colistener::MessageField>& fields,
                                    nlohmann::json& json_msg);
    static void deserialize_builtin_type(const uint8_t* buffer, size_t& offset,
                                         RosDataType type,
                                         nlohmann::json& value);

    static RosDataType convert_to_rostype(const std::string& type);

    template <typename T>
    static std::string vector_to_string(const std::vector<T>& vec) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << vec[i];
            if (i != vec.size() - 1) {
                oss << ", ";
            }
        }
        oss << "]";
        return oss.str();
    }
};
} // namespace ros1_listener

#endif // ROS1_LISTENER_LISTENER_H

