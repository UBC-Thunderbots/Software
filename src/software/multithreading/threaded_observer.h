#pragma once

#include <boost/bind.hpp>
#include <thread>

#include "software/multithreading/observer.h"

enum class ThreadedObserverOrdering
{
    QUEUE,  // first in, first out
    STACK   // last in, first out
};

/**
 * The general usage of this class should be to extend it, then override
 * `onValueReceived` with whatever custom functionality should occur when a new value
 * is received.
 *
 * @tparam T The type of object this class is observing
 * @tparam Ordering the ordering of which objects in the internal buffer are iterated
 *                  through when calling `onValueReceived`.
 */
template <typename T, ThreadedObserverOrdering Ordering>
class GenericThreadedObserver : public Observer<T>
{
   public:
    explicit GenericThreadedObserver(
        size_t buffer_size = Observer<T>::DEFAULT_BUFFER_SIZE);

    ~GenericThreadedObserver() override;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class uses
    GenericThreadedObserver &operator=(const GenericThreadedObserver &) = delete;
    GenericThreadedObserver(const GenericThreadedObserver &)            = delete;

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

template <typename T>
class ThreadedObserver
    : public GenericThreadedObserver<T, ThreadedObserverOrdering::STACK>
{
   public:
    ThreadedObserver<T>()
        : GenericThreadedObserver<T, ThreadedObserverOrdering::STACK>(){};
    explicit ThreadedObserver<T>(size_t buffer_size)
        : GenericThreadedObserver<T, ThreadedObserverOrdering::STACK>(buffer_size){};
};

template <typename T>
class OrderedThreadedObserver
    : public GenericThreadedObserver<T, ThreadedObserverOrdering::QUEUE>
{
   public:
    OrderedThreadedObserver<T>()
        : GenericThreadedObserver<T, ThreadedObserverOrdering::QUEUE>(){};
    explicit OrderedThreadedObserver<T>(size_t buffer_size)
        : GenericThreadedObserver<T, ThreadedObserverOrdering::QUEUE>(buffer_size){};
};

#include "software/multithreading/threaded_observer.tpp"
