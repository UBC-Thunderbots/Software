#pragma once

template <typename T>
GenericThreadedObserver<T>::GenericThreadedObserver(size_t buffer_size)
    : Observer<T>(buffer_size),
      in_destructor(false),
      IN_DESTRUCTOR_CHECK_PERIOD(Duration::fromSeconds(0.1))
{
    pull_from_buffer_thread = std::thread(
        boost::bind(&GenericThreadedObserver::continuouslyPullValuesFromBuffer, this));
}

template <typename T>
void GenericThreadedObserver<T>::onValueReceived(T val)
{
    // Do nothing, this function should be overriden to enable custom behavior on
    // message reception.
}

template <typename T>
void GenericThreadedObserver<T>::continuouslyPullValuesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();
        std::optional<T> new_val;

        new_val = this->getNextValue(IN_DESTRUCTOR_CHECK_PERIOD);

        if (new_val)
        {
            onValueReceived(*new_val);
        }

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

template <typename T>
GenericThreadedObserver<T>::~GenericThreadedObserver()
{
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    pull_from_buffer_thread.join();
}

template <typename T>
std::optional<T> FirstInFirstOutThreadedObserver<T>::getNextValue(
    const Duration& max_wait_time)
{
    return this->popLeastRecentlyReceivedValue(max_wait_time);
}

template <typename T>
std::optional<T> LastInFirstOutThreadedObserver<T>::getNextValue(
    const Duration& max_wait_time)
{
    return this->popLeastRecentlyReceivedValue(max_wait_time);
}
