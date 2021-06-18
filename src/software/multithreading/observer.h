#pragma once

#include "shared/constants.h"
#include "software/multithreading/thread_safe_buffer.h"

/**
 * This class observes an "Subject<T>". That is, it can be registered with an
 * "Subject<T>" to receive new instances of type T when they are available
 *
 * @tparam T The type of object this class is observing
 */
template <typename T>
class Observer
{
   public:
    Observer(size_t buffer_size = DEFAULT_BUFFER_SIZE);

    /**
     * Add the given value to the internal buffer
     *
     * @param val The value to add to the internal buffer
     */
    virtual void receiveValue(T val);

    /**
     * Calculate the data received per second using the internal time buffer
     *
     * @return the data received per second
     */
    virtual double getDataReceivedPerSecond() final;

    virtual ~Observer() = default;

    static constexpr size_t TIME_BUFFER_SIZE = 5;

   protected:
    /**
     * Pops the most recently received value and returns it
     *
     * If no value is available, this will block until:
     * - a value becomes available
     * - the given amount of time is exceeded
     * - the destructor of this class is called
     *
     * @param max_wait_time The maximum duration to wait for a new value before
     *                      returning
     *
     * @return The value most recently added to the buffer or std::nullopt if none is
     *         available
     */
    virtual std::optional<T> popMostRecentlyReceivedValue(Duration max_wait_time) final;

    /**
     * Pops the least recently received value and returns it
     *
     * If no value is available, this will block until:
     * - a value becomes available
     * - the given amount of time is exceeded
     * - the destructor of this class is called
     *
     * @param max_wait_time The maximum duration to wait for a new value before
     *                      returning
     *
     * @return The value least recently added to the buffer or std::nullopt if none is
     *         available
     */
    virtual std::optional<T> popLeastRecentlyReceivedValue(Duration max_wait_time) final;

    static constexpr size_t DEFAULT_BUFFER_SIZE = 1;

   private:
    ThreadSafeBuffer<T> buffer;
    boost::circular_buffer<std::chrono::milliseconds> receive_time_buffer;
};

#include "software/multithreading/observer.tpp"
