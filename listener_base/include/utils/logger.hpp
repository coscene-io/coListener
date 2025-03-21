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

#ifndef UTILS__LOGGER_HPP_
#define UTILS__LOGGER_HPP_

#include <string>
#include <fstream>
#include <mutex>
#include <cstring>
#include <cstdio>
#include <vector>

namespace colistener {

enum class LogLevel {
    DEBUG = 1,
    INFO = 2,
    WARN = 4,
    ERROR = 8
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(LogLevel level, const std::string& message);
    void set_log_dir(const std::string& dir);
    void set_log_level(LogLevel level);

private:
    std::string log_dir_;
    std::mutex mutex_;
    std::ofstream current_file_;
    std::string current_date_;
    LogLevel current_level_;

    void check_and_rotate_log();
    void clean_old_logs() const;
    static std::string get_level_string(LogLevel level);
    static std::string get_current_time_str();
    static std::string get_current_date_str();
    std::string get_log_file_name(const std::string& date) const;
    static bool should_log(LogLevel msg_level, LogLevel filter_level);
};

namespace detail {
    // Get filename only from full path
    inline const char* get_filename(const char* path) {
        const char* filename = strrchr(path, '/');
        return filename ? filename + 1 : path;
    }

    inline std::string format_string(const char* file, int line, const char* msg) {
        return std::string("[") + get_filename(file) + ":" + std::to_string(line) + "] " + msg;
    }

    template<typename... Args>
    std::string format_string(const char* file, int line, const char* format, Args... args) {
        int size = snprintf(nullptr, 0, format, args...) + 1;
        if (size <= 0) { return "Format Error"; }
        std::vector<char> buf(size);
        snprintf(buf.data(), size, format, args...);
        return std::string("[") + get_filename(file) + ":" + std::to_string(line) + "] " + 
               std::string(buf.data(), buf.data() + size - 1);
    }
}  // namespace detail

#define COLOG_INFO(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::INFO, \
        colistener::detail::format_string(__FILE__, __LINE__, __VA_ARGS__))

#define COLOG_WARN(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::WARN, \
        colistener::detail::format_string(__FILE__, __LINE__, __VA_ARGS__))

#define COLOG_ERROR(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::ERROR, \
        colistener::detail::format_string(__FILE__, __LINE__, __VA_ARGS__))

#define COLOG_DEBUG(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::DEBUG, \
        colistener::detail::format_string(__FILE__, __LINE__, __VA_ARGS__))

}  // namespace colistener

#endif  // UTILS__LOGGER_HPP_
