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


#ifndef COLISTENER_PERSISTENCE_DATABASE_MANAGER_H
#define COLISTENER_PERSISTENCE_DATABASE_MANAGER_H


#include <string>
#include <vector>
#include <mutex>
#include <sqlite3.h>
#include "colistener.hpp"
#include <chrono>
#include <utils/logger.hpp>

namespace colistener {
class DatabaseManager {
public:
    // static DatabaseManager& getInstance() {
    //     static DatabaseManager instance;
    //     return instance;
    // }

    DatabaseManager() = default;
    DatabaseManager(const DatabaseManager&) = delete;
    DatabaseManager& operator=(const DatabaseManager&) = delete;
    ~DatabaseManager();


    bool init(const std::string& db_path = "/tmp/colistener_persistence.db", int64_t expire_secs = 3600);
    bool insert_message(const MessageCache& message);
    bool remove_message(const MessageCache& message);

    bool remove_messages(const std::vector<MessageCache>& messages);
    // bool remove_messages(const std::unordered_map<int64_t, MessageCache>& messages);
    
    std::vector<MessageCache> get_all_messages();
    bool flush_cache();

private:
    bool create_table() const;
    sqlite3* db_{nullptr};
    int64_t expire_time_{0};
    std::mutex mutex_;
    std::vector<MessageCache> expired_messages_;
    std::vector<MessageCache> message_cache_;
    size_t cache_size_limit_{100};
    std::chrono::time_point<std::chrono::steady_clock> last_flush_time_;
    std::chrono::seconds flush_interval_{5};
};


} // namespace colistener

#endif //COLISTENER_PERSISTENCE_DATABASE_MANAGER_H
