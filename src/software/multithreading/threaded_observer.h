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
class ThreadedObserver : public Observer<T>
{
   public:
    ThreadedObserver();

    ~ThreadedObserver() override;

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

    // The period for checking whether or not the destructor for this class has
    // been called
    const Duration IN_DESTRUCTOR_CHECK_PERIOD;

    // This is the thread that will continuously pull values from the buffer
    // and pass them into the `onValueReceived`
    std::thread pull_from_buffer_thread;
};

#include "software/multithreading/threaded_observer.tpp"
