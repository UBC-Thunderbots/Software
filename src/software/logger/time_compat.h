#pragma once

#include <chrono>

#if defined(__APPLE__)
using Clock = std::chrono::system_clock;
#else
using Clock = std::chrono::_V2::system_clock;
#endif
