#pragma once

#include "shared/constants.h"
#include "software/multithreading/thread_safe_buffer.hpp"

/**
 * This class observes an "Subject<T>". That is, it can be registered with an
 * "Subject<T>" to receive new instances of type T when they are available
 *
 * @tparam T The type of object this class is observing
 * @tparam Clock A clock that satisfies the TrivialClock requirements
 */
template <typename T, typename Clock = std::chrono::steady_clock>
class Observer
{
   public:
    /**
     * Creates a new Observer
     *
     * @param buffer_size size of the buffer
     * @param log_buffer_full whether or not to log when the buffer is full
     */
    Observer(size_t buffer_size = DEFAULT_BUFFER_SIZE, bool log_buffer_full = true);

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

template <typename T, typename Clock>
Observer<T, Clock>::Observer(size_t buffer_size, bool log_buffer_full)
    : buffer(buffer_size, log_buffer_full), receive_time_buffer(TIME_BUFFER_SIZE)
{
}

template <typename T, typename Clock>
void Observer<T, Clock>::receiveValue(T val)
{
    receive_time_buffer.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()));
    buffer.push(std::move(val));
}

template <typename T, typename Clock>
std::optional<T> Observer<T, Clock>::popMostRecentlyReceivedValue(Duration max_wait_time)
{
    return buffer.popMostRecentlyAddedValue(max_wait_time);
}

template <typename T, typename Clock>
std::optional<T> Observer<T, Clock>::popLeastRecentlyReceivedValue(Duration max_wait_time)
{
    return buffer.popLeastRecentlyAddedValue(max_wait_time);
}

template <typename T, typename Clock>
double Observer<T, Clock>::getDataReceivedPerSecond()
{
    if (receive_time_buffer.empty())
    {
        return 0;
    }
    else
    {
        double time_s =
            static_cast<double>(
                (receive_time_buffer.back() - receive_time_buffer.front()).count()) *
            SECONDS_PER_MILLISECOND;

        double rate = static_cast<double>(receive_time_buffer.size() - 1) / time_s;
        return std::max(0.0, rate);
    }
}
