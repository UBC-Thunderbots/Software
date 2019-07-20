#pragma once

#include <cstddef>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

/**
 * This class represents a buffer of objects
 *
 * It's public API is fully thread-safe (meaning that it can be accessed by
 * multiple threads at the same time without issue)
 *
 * @tparam T The type of whatever is being buffered
 */
template <typename T>
class ThreadSafeBuffer {
public:
    explicit ThreadSafeBuffer(std::size_t buffer_size);

    // Copying this class is not permitted
    ThreadSafeBuffer(const ThreadSafeBuffer&) = delete;

    /**
     * Removes the least recently value added to the buffer and returns it
     *
     * If the buffer is empty, this function will *block* until one becomes available
     *
     * @return The least recently value added to the buffer and returns it
     */
    T pullLeastRecentlyAddedValue();

    /**
     * Removes the most recently value added to the buffer and returns it
     *
     * If the buffer is empty, this function will *block* until one becomes available
     *
     * @return The most recently value added to the buffer and returns it
     */
    T pullMostRecentlyAddedValue();

    /**
     * Push the given value onto the buffer
     *
     * If the buffer is already full, this will overwrite the least recently added value
     *
     * @param value The value to push onto the buffer
     */
    void push(const T& value);

private:

    /**
     * Waits for the buffer to have at least one value
     *
     * @return A unique_lock on the contents of the buffer
     */
    std::unique_lock<std::mutex> waitForBufferToHaveAValue();

    std::mutex buffer_mutex;
    boost::circular_buffer<T> buffer;
};

template<typename T>
ThreadSafeBuffer<T>::ThreadSafeBuffer(std::size_t buffer_size) : buffer(buffer_size) {
}

template<typename T>
T ThreadSafeBuffer<T>::pullLeastRecentlyAddedValue() {
    auto buffer_lock = waitForBufferToHaveAValue();

    T result = buffer.front();
    buffer.pop_front();
    return result;
}

template<typename T>
T ThreadSafeBuffer<T>::pullMostRecentlyAddedValue() {
    auto buffer_lock = waitForBufferToHaveAValue();

    T result = buffer.back();
    buffer.pop_back();
    return result;
}

template<typename T>
void ThreadSafeBuffer<T>::push(const T& value) {
    std::scoped_lock<std::mutex> buffer_lock(buffer_mutex);
    buffer.push_back(value);
}

template<typename T>
std::unique_lock<std::mutex> ThreadSafeBuffer<T>::waitForBufferToHaveAValue() {
    std::condition_variable cv;
    std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
    cv.wait(buffer_lock, [this]{return !buffer.empty();});

    // NOTE: We need to return this in order to prevent it being destructed and
    //       prevent it being written to before the value is read
    return buffer_lock;
}

