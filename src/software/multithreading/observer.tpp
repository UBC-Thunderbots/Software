#pragma once

template <typename T>
Observer<T>::Observer(size_t buffer_size) : buffer(buffer_size), time_buffer(TIME_BUFFER_SIZE, false)
{
}

template <typename T>
void Observer<T>::receiveValue(T val)
{
    time_buffer.push(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()));
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
int Observer<T>::getDataReceivedPerSecond()
{
    std::vector<std::chrono::milliseconds> times;

    for (unsigned int i = 0; i < TIME_BUFFER_SIZE; i++)
    {
        times.push_back(*time_buffer.popLeastRecentlyAddedValue());
    }

    auto time = (times.back() - times.front());
    double seconds = (double)(time.count())/1000;
    return seconds > 0 ? int(TIME_BUFFER_SIZE/seconds) : 0;
}
