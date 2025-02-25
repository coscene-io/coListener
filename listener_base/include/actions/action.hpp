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

#ifndef COLISTENER_ACTIONS_ACTION_H
#define COLISTENER_ACTIONS_ACTION_H

#include <utils/json.hpp>
#include <string>
#include <vector>
#include "colistener.hpp"

namespace colistener {

class Action {
public:
    virtual ~Action() = default;
    virtual bool execute(const std::vector<MessageCache>& messages) = 0;

    static std::shared_ptr<Action> create(const std::string& type);
};

} // namespace colistener

#endif // COLISTENER_ACTIONS_ACTION_H
