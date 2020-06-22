#pragma once

template <typename T>
Observer<T>::Observer() : buffer(DEFAULT_BUFFER_SIZE, false)
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
