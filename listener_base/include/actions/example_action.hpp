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

#ifndef ACTIONS__EXAMPLE_ACTION_HPP_
#define ACTIONS__EXAMPLE_ACTION_HPP_

#include "actions/action.hpp"
#include <vector>

namespace colistener {
class ExampleAction final : public Action {
public:
    ExampleAction();
    ~ExampleAction() override;
    bool execute(const std::vector<MessageCache>& messages) override;
};

}


#endif  // ACTIONS__EXAMPLE_ACTION_HPP_
