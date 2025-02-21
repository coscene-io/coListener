#include "utils/logger.hpp"

#include <colistener.hpp>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <dirent.h>

namespace colistener {
Logger::Logger() : log_dir_("/tmp/colistener/logs/"), current_level_(LogLevel::INFO) {
    createDirectory(log_dir_);
}

Logger::~Logger() {
    if (current_file_.is_open()) {
        current_file_.close();
    }
}

void Logger::set_log_dir(const std::string& dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_dir_ = dir;
    if (log_dir_.back() != '/') {
        log_dir_ += '/';
    }
    createDirectory(log_dir_);
}

void Logger::set_log_level(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_level_ = level;
}

bool Logger::should_log(LogLevel msg_level, LogLevel filter_level) {
    return static_cast<int>(msg_level) >= static_cast<int>(filter_level);
}

void Logger::log(const LogLevel level, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!should_log(level, current_level_)) {
        return;
    }

    check_and_rotate_log();

    if (current_file_.is_open()) {
        current_file_ << get_current_time_str() << " "
            << "[" << get_level_string(level) << "] "
            << message << std::endl;
        current_file_.flush();
    }
}

void Logger::check_and_rotate_log() {
    const std::string date = get_current_date_str();
    if (date != current_date_ || !current_file_.is_open()) {
        if (current_file_.is_open()) {
            current_file_.close();
        }

        current_date_ = date;
        const std::string filename = get_log_file_name(date);
        current_file_.open(filename, std::ios::app);

        clean_old_logs();
    }
}

void Logger::clean_old_logs() const {
    const int max_days = 7;
    DIR* dir = opendir(log_dir_.c_str());
    if (dir == nullptr) {
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG) {
            std::string filepath = log_dir_ + entry->d_name;
            struct stat file_stat{};
            if (stat(filepath.c_str(), &file_stat) == 0) {
                const auto now = std::time(nullptr);
                const double days = difftime(now, file_stat.st_mtime) / (60 * 60 * 24);

                if (days >= max_days) {
                    remove(filepath.c_str());
                }
            }
        }
    }
    closedir(dir);
}

std::string Logger::get_level_string(const LogLevel level) {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return " INFO";
        case LogLevel::WARN: return " WARN";
        case LogLevel::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

std::string Logger::get_current_time_str() {
    const auto now = std::chrono::system_clock::now();
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;

    const auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << "." << std::setfill('0') << std::setw(3) << now_ms;
    return ss.str();
}

std::string Logger::get_current_date_str() {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d");
    return ss.str();
}

std::string Logger::get_log_file_name(const std::string& date) const {
    return log_dir_ + "colistener_" + date + ".log";
}
} // namespace colistener
