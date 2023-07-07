#pragma once

#include <boost/circular_buffer.hpp>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>

#include "software/logger/logger.h"
#include "software/time/duration.h"
#include "software/util/typename/typename.h"

/**
 * This class represents a buffer of objects
 *
 * It's public API is fully thread-safe (meaning that it can be accessed by
 * multiple threads at the same time without issue)
 *
 * @tparam T The type of whatever is being buffered
 */
template <typename T>
class ThreadSafeBuffer
{
   public:
    // Force the user to specify a size
    explicit ThreadSafeBuffer() = delete;

    /**
     * Creates a new ThreadSafeBuffer
     *
     * @param buffer_size size of the buffer
     * @param log_buffer_full whether or not to log when the buffer is full
     */
    explicit ThreadSafeBuffer(std::size_t buffer_size, bool log_buffer_full = true);

    // Copying this class is not permitted
    ThreadSafeBuffer(const ThreadSafeBuffer&) = delete;

    /**
     * Removes the value least recently added to the buffer and returns it
     *
     * ex. if A,B,C were added to the buffer (in that order), this would return A
     *
     * If the buffer is empty, this function will *block* until:
     * - a value becomes available
     * - the given amount of time is exceeded
     * - the destructor of this class is called
     *
     * @param max_wait_time The maximum duration to wait for a new value before
     *                      returning
     *
     * @return The most recently added value to the buffer, or std::nullopt if none is
     *         available
     */
    std::optional<T> popLeastRecentlyAddedValue(
        Duration max_wait_time = Duration::fromSeconds(0));

    /**
     * Removes the value most recently added to the buffer and returns it
     *
     * ex. if A,B,C were added to the buffer (in that order), this would return C
     *
     * If the buffer is empty, this function will *block* until:
     * - a value becomes available
     * - the given amount of time is exceeded
     * - the destructor of this class is called
     *
     * @param max_wait_time The maximum duration to wait for a new value before
     *                      returning
     *
     * @return The most recently added value to the buffer, or std::nullopt if none is
     *         available
     */
    std::optional<T> popMostRecentlyAddedValue(
        Duration max_wait_time = Duration::fromSeconds(0));

    /**
     * Push the given value onto the buffer
     *
     * If the buffer is already full, this will overwrite the least recently added value
     *
     * @param value The value to push onto the buffer
     */
    void push(const T& value);

    ~ThreadSafeBuffer();

   private:
    /**
     * Waits for the buffer to have at least one value
     *
     * @param max_wait_time The maximum duration to wait for a new value before
     *                      returning
     *
     * @return A unique_lock on the contents of the buffer
     */
    std::unique_lock<std::mutex> waitForBufferToHaveAValue(Duration max_wait_time);

    std::mutex buffer_mutex;
    boost::circular_buffer<T> buffer;

    std::condition_variable received_new_value;

    std::mutex destructor_called_mutex;
    bool log_buffer_full;
    bool destructor_called;
};

template <typename T>
ThreadSafeBuffer<T>::ThreadSafeBuffer(std::size_t buffer_size, bool log_buffer_full)
    : buffer(buffer_size), log_buffer_full(log_buffer_full), destructor_called(false)
{
}

template <typename T>
std::optional<T> ThreadSafeBuffer<T>::popLeastRecentlyAddedValue(Duration max_wait_time)
{
    // We hold the returned lock in a variable here so that we hold the lock on the
    // buffer mutex until the lock is destructed at the end of this function
    auto buffer_lock = waitForBufferToHaveAValue(max_wait_time);

    std::optional<T> result = std::nullopt;
    if (!buffer.empty())
    {
        result = buffer.front();
        buffer.pop_front();
    }
    return result;
}

template <typename T>
std::optional<T> ThreadSafeBuffer<T>::popMostRecentlyAddedValue(Duration max_wait_time)
{
    // We hold the returned lock in a variable here so that we hold the lock on the
    // buffer mutex until the lock is destructed at the end of this function
    auto buffer_lock = waitForBufferToHaveAValue(max_wait_time);

    std::optional<T> result = std::nullopt;
    if (!buffer.empty())
    {
        result = buffer.back();
        buffer.pop_back();
    }
    return result;
}

template <typename T>
void ThreadSafeBuffer<T>::push(const T& value)
{
    std::scoped_lock<std::mutex> buffer_lock(buffer_mutex);
    if (log_buffer_full && buffer.full())
    {
        //        LOG(WARNING) << "Pushing to a full ThreadSafeBuffer of type: " <<
        //        TYPENAME(T)
        //                     << std::endl;
    }
    buffer.push_back(value);
    received_new_value.notify_all();
}

template <typename T>
std::unique_lock<std::mutex> ThreadSafeBuffer<T>::waitForBufferToHaveAValue(
    Duration max_wait_time)
{
    std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
    received_new_value.wait_for(
        buffer_lock, std::chrono::duration<float>(max_wait_time.toSeconds()), [this] {
            std::scoped_lock destructor_called_lock(destructor_called_mutex);
            return !buffer.empty() || destructor_called;
        });

    // NOTE: We need to return this in order to prevent it being destructed so
    //       the lock is maintained until the value is read
    return buffer_lock;
}

template <typename T>
ThreadSafeBuffer<T>::~ThreadSafeBuffer()
{
    destructor_called_mutex.lock();
    destructor_called = true;
    destructor_called_mutex.unlock();

    received_new_value.notify_all();
}
