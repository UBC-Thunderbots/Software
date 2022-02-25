#pragma once

#include <optional>

#include "software/multithreading/threaded_observer.hpp"

/**
 * The general usage of this class should be to extend it, then override
 * `onValueReceived` with whatever custom functionality should occur when a new value
 * is received. This class will call `onValueReceived` with objects in the internal
 * buffer in a first in, first out order.
 *
 * @tparam T The type of object this class is observing
 */
template <typename T>
class FirstInFirstOutThreadedObserver : public ThreadedObserver<T>
{
   public:
    FirstInFirstOutThreadedObserver() : ThreadedObserver<T>(){};

    /**
     * Creates a new FirstInFirstOutThreadedObserver
     *
     * @param buffer_size size of the buffer
     * @param log_buffer_full whether or not to log when the buffer is full
     */
    explicit FirstInFirstOutThreadedObserver<T>(size_t buffer_size,
                                                bool log_buffer_full = true)
        : ThreadedObserver<T>(buffer_size, log_buffer_full){};
    std::optional<T> getNextValue(const Duration& max_wait_time) final override;
};

template <typename T>
std::optional<T> FirstInFirstOutThreadedObserver<T>::getNextValue(
    const Duration& max_wait_time)
{
    return this->popLeastRecentlyReceivedValue(max_wait_time);
}
