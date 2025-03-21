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

#include <iostream>
#include <vector>
#include "actions/example_action.hpp"
#include "utils/logger.hpp"

namespace colistener {

ExampleAction::ExampleAction() {
    // construct action
}

ExampleAction::~ExampleAction() {
    // deconstruct action here
}

bool ExampleAction::execute(const std::vector<MessageCache>& messages) {
    // You can process the data here by either sending it via HTTP, storing it on the local disk, or doing anything else you want.
    // Of course, you can also perform secondary processing on the data here.
    // The data in the parameter has already been persisted.
    // If this function returns true, all the messages in the parameters will be removed from the persistent database.
    // If it returns false, the data will remain in the persistent database,
    // and the existing data will appear again in the parameter when the function is called next time.

    if (messages.empty()) return true;

    nlohmann::json json_array = nlohmann::json::array();
    for (const auto& cache_item : messages) {
        nlohmann::json item;
        item["topic"] = cache_item.topic;
        item["msg"] = nlohmann::json::parse(cache_item.msg);
        item["msgType"] = cache_item.msgType;
        item["ts"] = cache_item.ts;
        json_array.push_back(item);
    }
    COLOG_INFO("batch send messages: %s", json_array.dump().c_str());
    return true;
}
}  // namespace colistener
