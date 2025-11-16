#pragma once

#include <chrono>

#if defined(__APPLE__)
using Clock = std::chrono::system_clock;
#else
using Clock = std::chrono::_V2::system_clock;
#endif

#if __cplusplus > 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

