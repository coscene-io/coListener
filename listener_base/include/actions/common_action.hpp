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

#ifndef COLISTENER_ACTIONS_COMMON_ACTION_H
#define COLISTENER_ACTIONS_COMMON_ACTION_H

#include "actions/action.hpp"
#include <curl/curl.h>

namespace colistener {

class CommonAction final : public Action {
public:
    CommonAction();
    ~CommonAction() override;
    bool execute(const std::vector<MessageCache>& messages) override;

private:
    CURL* curl_;
    std::string endpoint_;
    struct curl_slist* headers_;
    FILE* dev_null_;
};

} // namespace colistener

#endif // COLISTENER_ACTIONS_COMMON_ACTION_H
