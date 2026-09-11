#pragma once
#include "tools/logger.hpp"
namespace bt {
inline std::shared_ptr<spdlog::logger> logger = logger::create_colored_logger("decision");
}