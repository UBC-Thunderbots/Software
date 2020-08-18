#pragma once

template <typename T>
Observer<T>::Observer(size_t buffer_size) : buffer(buffer_size)
{
}

template <typename T>
void Observer<T>::receiveValue(T val)
{
    buffer.push(std::move(val));
}

template <typename T>
std::optional<T> Observer<T>::popMostRecentlyReceivedValue(Duration max_wait_time)
{
    return buffer.popMostRecentlyAddedValue(max_wait_time);
}

template <typename T>
std::optional<T> Observer<T>::popLeastRecentlyReceivedValue(Duration max_wait_time)
{
    return buffer.popLeastRecentlyAddedValue(max_wait_time);
}
