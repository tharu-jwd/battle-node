#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <memory>

namespace SensorFusion
{

    enum class LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    class Logger
    {
    public:
        static Logger &getInstance();

        void setLogLevel(LogLevel level);
        void setLogFile(const std::string &filename);

        void log(LogLevel level, const std::string &message);
        void debug(const std::string &message);
        void info(const std::string &message);
        void warning(const std::string &message);
        void error(const std::string &message);

    private:
        Logger();
        ~Logger();
        Logger(const Logger &) = delete;
        Logger &operator=(const Logger &) = delete;

        std::string levelToString(LogLevel level);
        std::string getCurrentTimestamp();

        LogLevel minLevel_;
        std::ofstream logFile_;
        std::mutex mutex_;
    };

} // namespace SensorFusion

#endif // LOGGER_H
