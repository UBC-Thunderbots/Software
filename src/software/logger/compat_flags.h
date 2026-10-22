#pragma once

#include <chrono>

using Clock = std::chrono::system_clock;

#if __cplusplus > 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
