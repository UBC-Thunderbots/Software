#pragma once
#include "software/multithreading/threaded_observer.hpp"

/**
 * The general usage of this class should be to extend it, then override
 * `onValueReceived` with whatever custom functionality should occur when a new value
 * is received. This class will call `onValueReceived` with objects in the internal
 * buffer in a last in, first out order.
 *
 * @tparam T The type of object this class is observing
 */
template <typename T>
class LastInFirstOutThreadedObserver : public ThreadedObserver<T>
{
   public:
    LastInFirstOutThreadedObserver() : ThreadedObserver<T>(){};
    explicit LastInFirstOutThreadedObserver<T>(size_t buffer_size)
        : ThreadedObserver<T>(buffer_size){};
    std::optional<T> getNextValue(const Duration& max_wait_time) final;
};

template <typename T>
std::optional<T> LastInFirstOutThreadedObserver<T>::getNextValue(
    const Duration& max_wait_time)
{
    return this->popMostRecentlyReceivedValue(max_wait_time);
}
