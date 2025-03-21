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


#include "persistence/database_manager.hpp"
#include "utils/logger.hpp"
#include <mutex>
#include <ctime>
#include <sstream>
#include <string>
#include <vector>

namespace colistener {
bool DatabaseManager::init(const std::string& db_path, int64_t expire_secs) {
    createDirectory(db_path);
    expire_time_ = expire_secs;
    const int config_rc = sqlite3_config(SQLITE_CONFIG_MULTITHREAD);
    if (config_rc != SQLITE_OK) {
        return false;
    }

    const int init_rc = sqlite3_initialize();
    if (init_rc != SQLITE_OK) {
        return false;
    }

    const int rc = sqlite3_open(db_path.c_str(), &db_);
    if (rc) {
        return false;
    }
    return create_table();
}

bool DatabaseManager::create_table() const {
    const auto check_table_sql =
        "SELECT name FROM sqlite_master WHERE type='table' AND name='messages';";
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, check_table_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        return false;
    }

    bool table_exists = false;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        table_exists = true;
    }
    sqlite3_finalize(stmt);

    if (!table_exists) {
        const auto create_table_sql =
            "CREATE TABLE messages ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT,"
            "topic TEXT NOT NULL,"
            "message TEXT NOT NULL,"
            "datatype TEXT NOT NULL,"
            "timestamp REAL NOT NULL"
            ");";
        char* err_msg = nullptr;
        rc = sqlite3_exec(db_, create_table_sql, nullptr, nullptr, &err_msg);
        if (rc != SQLITE_OK) {
            sqlite3_free(err_msg);
            return false;
        }
    }
    return true;
}

bool DatabaseManager::insert_message(const MessageCache& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    message_cache_.push_back(message);

    auto now = std::chrono::steady_clock::now();
    bool should_flush =
        message_cache_.size() >= cache_size_limit_ ||
        (now - last_flush_time_) > flush_interval_;

    if (should_flush) {
        return flush_cache();
    }

    return true;
}

bool DatabaseManager::flush_cache() {
    if (message_cache_.empty()) {
        return true;
    }

    char* err_msg = nullptr;
    if (sqlite3_exec(db_, "BEGIN TRANSACTION", nullptr, nullptr, &err_msg) != SQLITE_OK) {
        COLOG_ERROR("Failed to begin transaction: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    const auto insert_sql = 
        "INSERT INTO messages (topic, message, datatype, timestamp) VALUES (?, ?, ?, ?);";
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, insert_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
        return false;
    }
    
    bool success = true;
    for (const auto& message : message_cache_) {
        sqlite3_reset(stmt);
        sqlite3_bind_text(stmt, 1, message.topic.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, message.msg.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, message.msgType.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_double(stmt, 4, message.ts);
        
        rc = sqlite3_step(stmt);
        if (rc != SQLITE_DONE) {
            success = false;
            break;
        }
    }
    
    sqlite3_finalize(stmt);
    
    if (success) {
        rc = sqlite3_exec(db_, "COMMIT", nullptr, nullptr, &err_msg);
    } else {
        rc = sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, &err_msg);
    }
    
    if (rc != SQLITE_OK) {
        COLOG_ERROR("Transaction failed: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    message_cache_.clear();
    last_flush_time_ = std::chrono::steady_clock::now();

    return success;
}

bool DatabaseManager::remove_message(const MessageCache& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto delete_sql = "DELETE FROM messages WHERE id = ?;";
    sqlite3_stmt* stmt;
    int rc = sqlite3_prepare_v2(db_, delete_sql, -1, &stmt, nullptr);
    if (rc != SQLITE_OK) {
        return false;
    }

    sqlite3_bind_int64(stmt, 1, message.id);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return rc == SQLITE_DONE;
}

bool DatabaseManager::remove_messages(const std::vector<MessageCache>& messages) {
    if (messages.empty()) {
        return true; // If no messages to delete, return success immediately
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (messages.size() <= 500) {
        std::stringstream sql;
        sql << "DELETE FROM messages WHERE id IN (";
        
        for (size_t i = 0; i < messages.size(); ++i) {
            if (i > 0) sql << ",";
            sql << messages[i].id;
        }
        sql << ");";
        
        char* err_msg = nullptr;
        int rc = sqlite3_exec(db_, sql.str().c_str(), nullptr, nullptr, &err_msg);
        
        if (rc != SQLITE_OK) {
            COLOG_ERROR("Batch deletion failed: %s", err_msg ? err_msg : "Unknown error");
            sqlite3_free(err_msg);
            return false;
        }
        
        COLOG_INFO("Successfully deleted %zu messages in batch", messages.size());
        return true;
    } else {
        const auto delete_sql = "DELETE FROM messages WHERE id = ?;";
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, delete_sql, -1, &stmt, nullptr);
        
        if (rc != SQLITE_OK) {
            COLOG_ERROR("Failed to prepare delete statement: %s", sqlite3_errmsg(db_));
            return false;
        }
        
        bool success = true;
        int64_t processed = 0;
        
        for (const auto& message : messages) {
            sqlite3_reset(stmt);
            sqlite3_bind_int64(stmt, 1, message.id);
            
            rc = sqlite3_step(stmt);
            if (rc != SQLITE_DONE) {
                COLOG_ERROR("Failed to delete message ID %lld: %s", message.id, sqlite3_errmsg(db_));
                success = false;
                break;
            }
            
            processed++;
            
            // Log progress every 500 records
            if (processed % 500 == 0) {
                COLOG_INFO("Processed %d/%zu messages", processed, messages.size());
            }
        }
        
        sqlite3_finalize(stmt);
        COLOG_INFO("Deletion operation completed, success: %s, processed %d/%zu messages", 
                 success ? "yes" : "no", processed, messages.size());
        return success;
    }
}

std::vector<MessageCache> DatabaseManager::get_all_messages() {
    std::vector<MessageCache> messages;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        const auto expired_time = std::time(nullptr) - expire_time_;
        for (const auto& msg : message_cache_) {
            if (msg.ts > expired_time) {
                messages.push_back(msg);
            } else {
                expired_messages_.push_back(msg);
            }
        }
        
        const auto select_sql = "SELECT id, topic, message, datatype, timestamp FROM messages;";
        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db_, select_sql, -1, &stmt, nullptr);
        if (rc != SQLITE_OK) {
            return messages;
        }

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            const auto id = sqlite3_column_int64(stmt, 0);
            const auto topic = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
            const auto msg = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
            const auto msgType = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3));
            const auto ts = sqlite3_column_double(stmt, 4);
            
            if (ts > expired_time) {
                messages.emplace_back(id, topic, msg, msgType, ts);
            } else {
                expired_messages_.emplace_back(id, topic, msg, msgType, ts);
            }
        }

        sqlite3_finalize(stmt);
    }

    if (!expired_messages_.empty()) {
        remove_messages(expired_messages_);
        expired_messages_.clear();
    }

    return messages;
}

DatabaseManager::~DatabaseManager() {
    if (db_) {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}
} // namespace colistener
