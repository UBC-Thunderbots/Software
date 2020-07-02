#pragma once

#include <boost/bind.hpp>
#include <thread>

#include "software/multithreading/observer.h"

/**
 * The general usage of this class should be to extend it, then override
 * `onValueReceived` with whatever custom functionality should occur when a new value
 * is received.
 *
 * @tparam T The type of object this class is observing
 */
template <typename T>
class OrderedThreadedObserver : public Observer<T>
{
   public:
    OrderedThreadedObserver();

    ~OrderedThreadedObserver() override;

   private:
    /**
     * This function will be called with a new value as it is received.
     *
     * If this function has not finished and a new value is received, then the next
     * value provided to it will be the newest value available *when this function
     * finishes*.
     *
     * Any class that extends this one should override this function with it's own
     * implementation. The implementation in this class does nothing (it's solely there
     * to deal with issues where this function is called before class construction
     * is fully complete, as it is run in a thread that is constructed in this class,
     * and so can start running before the subclass has been created).
     *
     * @param val The new value that has been received
     */
    virtual void onValueReceived(T val);

    /**
     * This function will run until the destructor of this class is called.
     * It will continuously pull values from the buffer and call `onValueReceived` with
     * them.
     * This is intended to be run in a separate thread.
     */
    void continuouslyPullValuesFromBuffer();

    // This indicates if the destructor of this class has been called
    std::mutex in_destructor_mutex;
    bool in_destructor;

    // The period for waiting for a message before it is destroyed
    const Duration IN_DESTRUCTOR_CHECK_PERIOD;

    // This is the thread that will continuously pull values from the buffer
    // and pass them into the `onValueReceived`
    std::thread pull_from_buffer_thread;
};

template <typename T>
OrderedThreadedObserver<T>::OrderedThreadedObserver()
    : in_destructor(false), IN_DESTRUCTOR_CHECK_PERIOD(Duration::fromSeconds(0.1))
{
    pull_from_buffer_thread = std::thread(
        boost::bind(&OrderedThreadedObserver::continuouslyPullValuesFromBuffer, this));
}

template <typename T>
void OrderedThreadedObserver<T>::onValueReceived(T val)
{
    // Do nothing, this function should be overriden to enable custom behavior on
    // message reception.
}

template <typename T>
void OrderedThreadedObserver<T>::continuouslyPullValuesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();

        std::optional<T> new_val =
            this->popLeastRecentlyReceivedValue(IN_DESTRUCTOR_CHECK_PERIOD);

        if (new_val)
        {
            onValueReceived(*new_val);
        }

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

template <typename T>
OrderedThreadedObserver<T>::~OrderedThreadedObserver()
{
    // TODO: possibly do something more intelligent here
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();
    pull_from_buffer_thread.join();
}
