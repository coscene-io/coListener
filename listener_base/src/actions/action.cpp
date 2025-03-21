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

#include <memory>
#include <string>
#include "actions/action.hpp"
#include "actions/common_action.hpp"
#include "actions/example_action.hpp"

namespace colistener {
std::shared_ptr<Action> Action::create(const std::string& type) {
    if (type == "common" || type.empty()) {
        return std::make_shared<CommonAction>();
    } else if (type == "example") {
        return std::make_shared<ExampleAction>();
    }
    // more action types here.

    throw std::runtime_error("Unknown action type: " + type);
}
} // namespace colistener
