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

#ifndef COLISTENER_H
#define COLISTENER_H
#include <string>
#include <vector>
#include <sstream>
#include <nlohmann/json.hpp>

namespace colistener {
extern const char VERSION[];
extern const char GIT_HASH[];

constexpr char DEFAULT_URL[] = "http://localhost";
constexpr char DEFAULT_PORT[] = "22524";
constexpr char DEFAULT_ROUTE[] = "/ruleEngine/messages";

enum class RosDataType
{
    RosDataTypeFloat = 1,
    RosDataTypeDouble = 2,
    RosDataTypeLongDouble = 3,
    RosDataTypeChar = 4,
    RosDataTypeWchar = 5,
    RosDataTypeBool = 6,
    RosDataTypeOctet = 7,
    RosDataTypeUint8 = 8,
    RosDataTypeInt8 = 9,
    RosDataTypeUint16 = 10,
    RosDataTypeInt16 = 11,
    RosDataTypeUint32 = 12,
    RosDataTypeInt32 = 13,
    RosDataTypeUint64 = 14,
    RosDataTypeInt64 = 15,
    RosDataTypeString = 16,
    RosDataTypeWString = 17,
    RosDataTypeMessage = 18,
    RosDataTypeUnknown = 19,
};

struct MessageField {
    RosDataType type;
    std::string name;
    bool is_array;
    bool is_builtin;
    std::string package_name;
    std::vector<MessageField> sub_fields;
};

struct MessageCache {
    std::string topic;
    nlohmann::json msg;
    std::string msgType;
    double ts;
};

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


}


#endif //COLISTENER_H
