#pragma once

template <typename T>
Observer<T>::Observer(size_t buffer_size)
    : buffer(buffer_size), time_buffer(TIME_BUFFER_SIZE)
{
}

template <typename T>
void Observer<T>::receiveValue(T val)
{
    time_buffer.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()));
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

template <typename T>
double Observer<T>::getDataReceivedPerSecond()
{
    auto time_ms  = (time_buffer.back() - time_buffer.front());
    double time_s = static_cast<double>(time_ms.count()) / 1000;
    return time_s > 0 ? static_cast<double>(time_buffer.size()) / time_s : 0;
}
