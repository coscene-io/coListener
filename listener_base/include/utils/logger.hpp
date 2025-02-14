#ifndef COLISTENER_UTILS_LOGGER_H
#define COLISTENER_UTILS_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>

namespace colistener {

enum class LogLevel {
    INFO,
    WARN,
    ERROR
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

#define COLOG_INFO(msg)  colistener::Logger::getInstance().log(colistener::LogLevel::INFO, msg)
#define COLOG_WARN(msg)  colistener::Logger::getInstance().log(colistener::LogLevel::WARN, msg)
#define COLOG_ERROR(msg) colistener::Logger::getInstance().log(colistener::LogLevel::ERROR, msg)

} // namespace colistener

#endif // COLISTENER_UTILS_LOGGER_H 