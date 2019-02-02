#pragma once

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>


namespace USB
{
    /**
     * Checks a call into libusb if it returned an error, and throws an exception if so.
     */
    long check_fn(const char *call, long err, unsigned int endpoint);
}  // namespace USB
