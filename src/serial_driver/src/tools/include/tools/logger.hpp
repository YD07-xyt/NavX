#pragma once
#include "fmt_eigen.hpp"
#include <memory>
#include <source_location>
#include <spdlog/common.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace logger {
// info 结构体（接受 logger）
template <typename... Args>
struct info {
    constexpr info(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::info,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
info(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> info<Args...>;

template <typename... Args>
struct warn {
    constexpr warn(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::warn,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
warn(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> warn<Args...>;


template <typename... Args>
struct error {
    constexpr error(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::err,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
error(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> error<Args...>;

template <typename... Args>
struct debug {
    constexpr debug(const std::shared_ptr<spdlog::logger>& log,
                   fmt::format_string<Args...> fmt,
                   Args&&... args,
                   std::source_location loc = std::source_location::current()) 
    {
        log->log(spdlog::source_loc{loc.file_name(),
                                    static_cast<int>(loc.line()),
                                    loc.function_name()},
                 spdlog::level::debug,
                 fmt,
                 std::forward<Args>(args)...);
    }
};
template <typename... Args>
debug(const std::shared_ptr<spdlog::logger>&,
     fmt::format_string<Args...> fmt,
     Args&&... args) -> debug<Args...>;

// 创建带整行颜色的 logger
inline std::shared_ptr<spdlog::logger> create_colored_logger(const std::string& name) {
    auto logger = spdlog::stdout_color_mt(name);
    //logger->set_pattern("%^ [%m-%d %H:%M:%S.%e] [%n] [%l] %v %$");
    logger->set_pattern("%^[%m-%d %H:%M:%S.%e] [%n] [%l] [%s:%#] %v%$");
    return logger;
}
    
}