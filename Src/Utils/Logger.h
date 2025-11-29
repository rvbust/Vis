/***********************************************************************
 **
 ** Copyright (c) 2012-2024 RVBUST Inc.
 **
 ** Permission is hereby granted, free of charge, to any person obtaining
 ** a copy of this software and associated documentation files (the
 ** "Software"), to deal in the Software without restriction, including
 ** without limitation the rights to use, copy, modify, merge, publish,
 ** distribute, sublicense, and/or sell copies of the Software, and to
 ** permit persons to whom the Software is furnished to do so, subject to
 ** the following conditions:
 **
 ** The above copyright notice and this permission notice shall be
 ** included in all copies or substantial portions of the Software.
 **
 ** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 ** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 ** MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 ** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 ** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 ** OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 ** WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ***********************************************************************/

#pragma once

#include <string>
#include <memory>

// Forward declare spdlog types
namespace spdlog {
    class logger;
}

namespace Vis {

/**
 * @brief Log level enumeration
 */
enum class LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Critical,
    Off
};

/**
 * @brief Logger wrapper for the Vis library.
 * 
 * This provides a centralized logging facility using spdlog under the hood.
 * The logger is thread-safe and supports multiple log levels.
 */
class Logger {
public:
    /// Get the singleton logger instance
    static Logger& instance();

    /// Set the global log level
    void setLevel(LogLevel level);

    /// Set the log level by name ("debug", "info", "warn", "error", "off")
    void setLevel(const std::string& levelName);

    /// Get the current log level
    LogLevel getLevel() const;

    /// Log at trace level
    template<typename... Args>
    void trace(const char* fmt, Args&&... args);

    /// Log at debug level
    template<typename... Args>
    void debug(const char* fmt, Args&&... args);

    /// Log at info level
    template<typename... Args>
    void info(const char* fmt, Args&&... args);

    /// Log at warn level
    template<typename... Args>
    void warn(const char* fmt, Args&&... args);

    /// Log at error level
    template<typename... Args>
    void error(const char* fmt, Args&&... args);

    /// Log at critical level
    template<typename... Args>
    void critical(const char* fmt, Args&&... args);

    /// Get the underlying spdlog logger (for advanced use)
    std::shared_ptr<spdlog::logger> getSpdLogger() const;

private:
    Logger();
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    class Impl;
    std::unique_ptr<Impl> m_impl;
};

/**
 * @brief Set the global log level.
 * @param levelName One of "trace", "debug", "info", "warn", "error", "critical", "off"
 */
void SetLogLevel(const std::string& levelName);

/**
 * @brief Get the library version string.
 */
const char* GetVersion();

}  // namespace Vis

//============================================================================
// Logging Macros
//============================================================================

// Helper macro to get function name
#ifndef VIS_FUNC_NAME
    #ifdef _MSC_VER
        #define VIS_FUNC_NAME __FUNCTION__
    #else
        #define VIS_FUNC_NAME __func__
    #endif
#endif

// Convenience logging macros
#define VIS_LOG_TRACE(...) ::Vis::Logger::instance().trace(__VA_ARGS__)
#define VIS_LOG_DEBUG(...) ::Vis::Logger::instance().debug(__VA_ARGS__)
#define VIS_LOG_INFO(...)  ::Vis::Logger::instance().info(__VA_ARGS__)
#define VIS_LOG_WARN(...)  ::Vis::Logger::instance().warn(__VA_ARGS__)
#define VIS_LOG_ERROR(...) ::Vis::Logger::instance().error(__VA_ARGS__)
#define VIS_LOG_CRITICAL(...) ::Vis::Logger::instance().critical(__VA_ARGS__)

