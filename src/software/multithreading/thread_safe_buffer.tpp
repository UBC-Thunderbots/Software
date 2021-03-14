#pragma once

#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

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
        LOG(WARNING) << "Pushing to a full ThreadSafeBuffer of type: "
                     << CLASS_TYPENAME(T) << std::endl;
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
