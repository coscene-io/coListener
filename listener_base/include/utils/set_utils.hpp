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

#ifndef UTILS__VECTOR_UTILS_HPP_
#define UTILS__VECTOR_UTILS_HPP_

#include <vector>
#include <unordered_set>
#include <algorithm>

namespace colistener {

template<typename T>
struct SetDiff {
    std::vector<T> missing;
    std::vector<T> added;
    
    bool isIdentical() const {
        return missing.empty() && added.empty();
    }
};

template<typename T>
SetDiff<T> findSetsDifference(const std::set<T>& A, const std::set<T>& B) {
    SetDiff<T> result;

    std::set_difference(A.begin(), A.end(), B.begin(), B.end(),
                        std::inserter(result.missing, result.missing.begin()));

    std::set_difference(B.begin(), B.end(), A.begin(), A.end(),
                        std::inserter(result.added, result.added.begin()));

    return result;
}
}  // namespace colistener

#endif  // UTILS__VECTOR_UTILS_HPP_
