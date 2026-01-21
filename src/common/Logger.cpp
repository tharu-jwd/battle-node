#include "common/Logger.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace SensorFusion
{

    Logger &Logger::getInstance()
    {
        static Logger instance;
        return instance;
    }

    Logger::Logger() : minLevel_(LogLevel::INFO) {}

    Logger::~Logger()
    {
        if (logFile_.is_open())
        {
            logFile_.close();
        }
    }

    void Logger::setLogLevel(LogLevel level)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        minLevel_ = level;
    }

    void Logger::setLogFile(const std::string &filename)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (logFile_.is_open())
        {
            logFile_.close();
        }
        logFile_.open(filename, std::ios::app);
    }

    void Logger::log(LogLevel level, const std::string &message)
    {
        if (level < minLevel_)
            return;

        std::lock_guard<std::mutex> lock(mutex_);

        std::string timestamp = getCurrentTimestamp();
        std::string levelStr = levelToString(level);

        std::string logMessage = "[" + timestamp + "] [" + levelStr + "] " + message;

        std::cout << logMessage << std::endl;

        if (logFile_.is_open())
        {
            logFile_ << logMessage << std::endl;
            logFile_.flush();
        }
    }

    void Logger::debug(const std::string &message) { log(LogLevel::DEBUG, message); }
    void Logger::info(const std::string &message) { log(LogLevel::INFO, message); }
    void Logger::warning(const std::string &message) { log(LogLevel::WARNING, message); }
    void Logger::error(const std::string &message) { log(LogLevel::ERROR, message); }

    std::string Logger::levelToString(LogLevel level)
    {
        switch (level)
        {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARNING:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
        }
    }

    std::string Logger::getCurrentTimestamp()
    {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

} // namespace SensorFusion
