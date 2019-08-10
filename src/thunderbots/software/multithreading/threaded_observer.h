#pragma once

#include <thread>

#include "multithreading/observer.h"

// TODO: rename to "ThreadedSubscriber"?

// TODO: better doc comment here. We need to be *very* clear
/**
 * This class is similar to "Observable<T>", except that it runs specified callback
 * functions with new "T" objects when they are received
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
    // TODO: Better function name and value name here
    // TODO: jdoc comment here
    virtual void newValueCallback(T val);

    // TODO: jdoc comment here
    void continuouslyPullValuesFromBuffer();

    // This indicates if the destructor of this class has been called
    std::mutex in_destructor_mutex;
    bool in_destructor;

    // This is the thread that will continuously pull values from the buffer
    // and pass them into the callback function
    std::thread pull_from_buffer_thread;
};

// TODO: implementation for this can maybe go in a `.cpp`?

template <typename T>
ThreadedObserver<T>::ThreadedObserver() : in_destructor(false),
pull_from_buffer_thread(boost::bind(&ThreadedObserver::continuouslyPullValuesFromBuffer, this))
{
}

template <typename T>
void ThreadedObserver<T>::newValueCallback(T val) {
    // TODO: this comment should be in the jdoc for this function?
    // Do nothing, this function should be overriden to enable custom behavior on
    // message reception.
}

template <typename T>
void ThreadedObserver<T>::continuouslyPullValuesFromBuffer()
{
    do
    {
        in_destructor_mutex.unlock();

        newValueCallback(this->getMostRecentValueFromBuffer());

        in_destructor_mutex.lock();
    } while (!in_destructor);
}

template <typename T>
ThreadedObserver<T>::~ThreadedObserver()
{
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // We must wait for the thread to stop, as if we destroy it while we're still
    // running we will segfault
    pull_from_buffer_thread.join();
}
