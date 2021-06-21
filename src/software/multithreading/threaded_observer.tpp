#pragma once

#include "software/multithreading/threaded_observer.h"

template <typename T>
ThreadedObserver<T>::ThreadedObserver(size_t buffer_size, bool log_buffer_full)
    : Observer<T>(buffer_size, log_buffer_full),
      in_destructor(false),
      IN_DESTRUCTOR_CHECK_PERIOD(Duration::fromSeconds(0.1))
{
    pull_from_buffer_thread = std::thread(
        boost::bind(&ThreadedObserver::continuouslyPullValuesFromBuffer, this));
}

template <typename T>
void ThreadedObserver<T>::onValueReceived(T val)
{
    // Do nothing, this function should be overridden to enable custom behavior on
    // message reception.
}

template <typename T>
void ThreadedObserver<T>::continuouslyPullValuesFromBuffer()
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
ThreadedObserver<T>::~ThreadedObserver()
{
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    pull_from_buffer_thread.join();
}

template <typename T>
std::optional<T> ThreadedObserver<T>::getNextValue(const Duration& max_wait_time)
{
    // Do nothing, this function should be overridden to enable custom behavior on
    // message reception.
    // We *must* provide an implementation here instead of making it pure virtual because
    // this function may be called from the `pull_from_buffer_thread` *before* the
    // subclass constructor has completed (ie. before it has "finished overriding" all the
    // methods). If this functions is pure virtual then this case would cause a crash.
    return std::nullopt;
}
