#ifndef COLISTENER_UTILS_LOGGER_H
#define COLISTENER_UTILS_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <sstream>
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

private:

    std::string log_dir_;
    std::mutex mutex_;
    std::ofstream current_file_;
    std::string current_date_;

    void check_and_rotate_log();
    void clean_old_logs() const;
    static std::string get_level_string(LogLevel level);
    static std::string get_current_time_str();
    static std::string get_current_date_str();
    std::string get_log_file_name(const std::string& date) const;
};

namespace detail {
    inline std::string format_string(const char* msg) {
        return std::string(msg);
    }

    template<typename... Args>
    std::string format_string(const char* format, Args... args) {
        int size = snprintf(nullptr, 0, format, args...) + 1;
        if (size <= 0) { return "Format Error"; }
        std::vector<char> buf(size);
        snprintf(buf.data(), size, format, args...);
        return std::string(buf.data(), buf.data() + size - 1);
    }
}

#define COLOG_INFO(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::INFO, \
        colistener::detail::format_string(__VA_ARGS__))

#define COLOG_WARN(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::WARN, \
        colistener::detail::format_string(__VA_ARGS__))

#define COLOG_ERROR(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::ERROR, \
        colistener::detail::format_string(__VA_ARGS__))

#define COLOG_DEBUG(...) \
    colistener::Logger::getInstance().log(colistener::LogLevel::INFO, \
        colistener::detail::format_string(__VA_ARGS__))

} // namespace colistener

#endif // COLISTENER_UTILS_LOGGER_H
