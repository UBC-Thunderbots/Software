#pragma once

template <typename T, ThreadedObserverOrdering Ordering>
TThreadedObserver<T, Ordering>::TThreadedObserver(size_t buffer_size)
    : Observer<T>(buffer_size),
      in_destructor(false),
      IN_DESTRUCTOR_CHECK_PERIOD(Duration::fromSeconds(0.1))
{
    pull_from_buffer_thread = std::thread(
        boost::bind(&TThreadedObserver::continuouslyPullValuesFromBuffer, this));
}

template <typename T, ThreadedObserverOrdering Ordering>
void TThreadedObserver<T, Ordering>::onValueReceived(T val)
{
    // Do nothing, this function should be overriden to enable custom behavior on
    // message reception.
}

template <typename T, ThreadedObserverOrdering Ordering>
void TThreadedObserver<T, Ordering>::continuouslyPullValuesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();
        std::optional<T> new_val;

        if constexpr (Ordering == ThreadedObserverOrdering::Queue)
        {
            new_val = this->popLeastRecentlyReceivedValue(IN_DESTRUCTOR_CHECK_PERIOD);
        }
        else
        {
            new_val = this->popMostRecentlyReceivedValue(IN_DESTRUCTOR_CHECK_PERIOD);
        }

        if (new_val)
        {
            onValueReceived(*new_val);
        }

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

template <typename T, ThreadedObserverOrdering Ordering>
TThreadedObserver<T, Ordering>::~TThreadedObserver()
{
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // We must wait for the thread to stop, as if we destroy it while it's still
    // running we will segfault
    pull_from_buffer_thread.join();
}
