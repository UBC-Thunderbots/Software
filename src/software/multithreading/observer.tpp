#pragma once

template <typename T>
Observer<T>::Observer(size_t buffer_size)
    : buffer(buffer_size), receive_time_buffer(TIME_BUFFER_SIZE)
{
}

template <typename T>
void Observer<T>::receiveValue(T val)
{
    receive_time_buffer.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
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
