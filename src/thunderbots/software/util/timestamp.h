#pragma once

#include <chrono>

typedef std::chrono::steady_clock::duration AITimestamp;

namespace Timestamp
{
    /**
     * Returns an AITimestamp of the current time
     *
     * @return an AITimestamp of the current time
     */
    static inline AITimestamp getTimestampNow()
    {
        return std::chrono::steady_clock::now().time_since_epoch();
    }

    /**
     * Returns the given timestamp in microseconds
     *
     * @param timestamp the timestamp to convert to microseconds
     *
     * @return the given timestamp in microseconds
     */
    static inline int64_t getMicroseconds(const AITimestamp& timestamp)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(timestamp).count();
    }
}  // namespace Timestamp
