#pragma once
#include "software/multithreading/threaded_observer.h"

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
    FirstInFirstOutThreadedObserver<T>() : ThreadedObserver<T>(){};
    explicit FirstInFirstOutThreadedObserver<T>(size_t buffer_size)
        : ThreadedObserver<T>(buffer_size){};
    std::optional<T> getNextValue(const Duration& max_wait_time) final;
};

template <typename T>
std::optional<T> FirstInFirstOutThreadedObserver<T>::getNextValue(
    const Duration& max_wait_time)
{
    return this->popLeastRecentlyReceivedValue(max_wait_time);
}
