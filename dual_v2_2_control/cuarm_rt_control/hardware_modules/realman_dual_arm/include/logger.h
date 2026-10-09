#ifndef LOGGER_H
#define LOGGER_H
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <filesystem>  
class Logger {
public:
    enum LogLevel { INFO, ERROR, DEBUG };

    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void log(LogLevel level, const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count()
            << " [" << levelToString(level) << "] " << message << std::endl;

        std::ofstream& file = getLogFile();
        file << oss.str();
        file.flush();

        // 仅ERROR级别输出到终端（可选，保留关键错误）
        if (level == ERROR) {
            std::cerr << oss.str();
        }
    }

private:
    Logger() {
        // 创建日志文件，带日期
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char filename[64];
        std::strftime(filename, sizeof(filename), "robot_log_%Y-%m-%d.txt", std::localtime(&now));
        logFile_.open(filename, std::ios::app);
        if (!logFile_.is_open()) {
            std::cerr << "Failed to open log file!" << std::endl;
        }
    }

    ~Logger() {
        if (logFile_.is_open()) {
            logFile_.close();
        }
    }

    std::ofstream& getLogFile() {
        return logFile_;
    }

    std::string levelToString(LogLevel level) {
        switch (level) {
            case INFO: return "INFO";
            case ERROR: return "ERROR";
            case DEBUG: return "DEBUG";
            default: return "UNKNOWN";
        }
    }

    std::ofstream logFile_;
    std::mutex mutex_;
};

#define LOG_INFO(msg) Logger::getInstance().log(Logger::INFO, msg)
#define LOG_ERROR(msg) Logger::getInstance().log(Logger::ERROR, msg)
#define LOG_DEBUG(msg) Logger::getInstance().log(Logger::DEBUG, msg)

#endif // LOGGER_H