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

#include <string>
#include <vector>
#include "actions/common_action.hpp"
#include "utils/logger.hpp"

namespace colistener {
CommonAction::CommonAction() {
    headers_["Content-Type"] = "application/json";
    headers_["User-Agent"] = "coListener/1.0";

    endpoint_ = std::string(DEFAULT_URL) +
        ":" +
        std::string(DEFAULT_PORT) +
        std::string(SEND_MESSAGES);
}

CommonAction::~CommonAction() = default;

bool CommonAction::execute(const std::vector<MessageCache>& messages) {
    if (messages.empty()) return true;

    // Log the number of messages to be processed
    COLOG_INFO("Processing %zu messages for sending", messages.size());

    // Create the root JSON object with "messages" array
    nlohmann::json root;
    root["messages"] = nlohmann::json::array();

    for (const auto& cache_item : messages) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        // Parse msg string into JSON object
        try {
            item["msg"] = nlohmann::json::parse(cache_item.msg);
        } catch (const nlohmann::json::parse_error& e) {
            COLOG_ERROR("Failed to parse msg as JSON: %s", e.what());
            // Fallback to empty object if parsing fails
            item["msg"] = nlohmann::json::object();
        }
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        root["messages"].push_back(item);
    }

    try {
        const std::string json_str = root.dump();
        
        // Log request details
        COLOG_DEBUG("Sending request to endpoint: %s", endpoint_.c_str());
        // Log detailed request payload for debugging
        COLOG_INFO("Request payload: %s", json_str.c_str());


        HttpResponse post_response = curl_client_.post(endpoint_, root, headers_);

        if (post_response.success) {
            return true;
        } else {
            COLOG_ERROR("POST request failed: %s", post_response.error_message.c_str());
            return false;
        }
    }
    catch (const std::exception& e) {
        // Log any exceptions that occur during request
        COLOG_ERROR("Exception occurred while sending request: %s", e.what());
        return false;
    }
}
} // namespace colistener
