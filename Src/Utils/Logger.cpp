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

#include "Logger.h"

#include <Vis/Export.h>

#define SPDLOG_STATIC_LIB 1
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <cctype>

namespace Vis {

//============================================================================
// Logger Implementation
//============================================================================

class Logger::Impl {
public:
    Impl() {
        m_logger = spdlog::get("Vis");
        if (!m_logger) {
            m_logger = spdlog::stdout_color_mt("Vis");
            m_logger->set_level(spdlog::level::err);
            m_logger->set_pattern("[%n][%^%L%$]: %v");
        }
    }

    ~Impl() = default;

    std::shared_ptr<spdlog::logger> m_logger;
    LogLevel m_level = LogLevel::Error;
};

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() : m_impl(std::make_unique<Impl>()) {}

Logger::~Logger() = default;

void Logger::setLevel(LogLevel level) {
    m_impl->m_level = level;
    
    spdlog::level::level_enum spdLevel;
    switch (level) {
        case LogLevel::Trace:    spdLevel = spdlog::level::trace; break;
        case LogLevel::Debug:    spdLevel = spdlog::level::debug; break;
        case LogLevel::Info:     spdLevel = spdlog::level::info; break;
        case LogLevel::Warn:     spdLevel = spdlog::level::warn; break;
        case LogLevel::Error:    spdLevel = spdlog::level::err; break;
        case LogLevel::Critical: spdLevel = spdlog::level::critical; break;
        case LogLevel::Off:      spdLevel = spdlog::level::off; break;
        default:                 spdLevel = spdlog::level::err; break;
    }
    
    m_impl->m_logger->set_level(spdLevel);
}

void Logger::setLevel(const std::string& levelName) {
    std::string lower = levelName;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    
    if (lower == "trace") {
        setLevel(LogLevel::Trace);
    } else if (lower == "debug") {
        setLevel(LogLevel::Debug);
    } else if (lower == "info") {
        setLevel(LogLevel::Info);
    } else if (lower == "warn" || lower == "warning") {
        setLevel(LogLevel::Warn);
    } else if (lower == "error" || lower == "err") {
        setLevel(LogLevel::Error);
    } else if (lower == "critical" || lower == "fatal") {
        setLevel(LogLevel::Critical);
    } else if (lower == "off" || lower == "none") {
        setLevel(LogLevel::Off);
    } else {
        m_impl->m_logger->warn("Unknown log level: {}, defaulting to error", levelName);
        setLevel(LogLevel::Error);
    }
}

LogLevel Logger::getLevel() const {
    return m_impl->m_level;
}

std::shared_ptr<spdlog::logger> Logger::getSpdLogger() const {
    return m_impl->m_logger;
}

// Template implementations
template<typename... Args>
void Logger::trace(const char* fmt, Args&&... args) {
    m_impl->m_logger->trace(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::debug(const char* fmt, Args&&... args) {
    m_impl->m_logger->debug(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::info(const char* fmt, Args&&... args) {
    m_impl->m_logger->info(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::warn(const char* fmt, Args&&... args) {
    m_impl->m_logger->warn(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::error(const char* fmt, Args&&... args) {
    m_impl->m_logger->error(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
void Logger::critical(const char* fmt, Args&&... args) {
    m_impl->m_logger->critical(fmt, std::forward<Args>(args)...);
}

// Explicit template instantiations for common types
template void Logger::trace(const char*);
template void Logger::debug(const char*);
template void Logger::info(const char*);
template void Logger::warn(const char*);
template void Logger::error(const char*);
template void Logger::critical(const char*);

//============================================================================
// Global Functions
//============================================================================

void SetLogLevel(const std::string& levelName) {
    Logger::instance().setLevel(levelName);
}

const char* GetVersion() {
    return VIS_VERSION_STRING;
}

}  // namespace Vis

