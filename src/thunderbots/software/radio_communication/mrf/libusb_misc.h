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


namespace USB {
    /**
     * These are for internal use only.
     */
    long check_fn(const char *call, long err, unsigned int endpoint);
}
