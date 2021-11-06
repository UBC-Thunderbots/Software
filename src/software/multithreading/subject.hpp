#pragma once

#include <vector>

#include "software/multithreading/observer.hpp"

/**
 * This class represents something that can be watched by an Observer.
 *
 * The general usage should be to pass "Observer<T>" objects into "registerObserver".
 * These "Observer<T>" objects will receive new data from this class when it is
 * available
 *
 * @tparam T The type of object that is being provided to all registered Observers
 */
template <typename T>
class Subject
{
   public:
    /**
     * Register the given observer with this class to receive new values when
     * they are available
     *
     * @param observer The observer to update with new values whenever they are
     *                 available
     */
    void registerObserver(std::shared_ptr<Observer<T>> observer);

    virtual ~Subject() = default;

   protected:
    /**
     * Sends the given value to all registered observers
     *
     * @param val The object to send to observers
     */
    virtual void sendValueToObservers(T val) final;

   private:
    // The observers that this class provides updates to
    std::vector<std::shared_ptr<Observer<T>>> observers;
};

template <typename T>
void Subject<T>::registerObserver(std::shared_ptr<Observer<T>> observer)
{
    observers.emplace_back(observer);
}

template <typename T>
void Subject<T>::sendValueToObservers(T val)
{
    for (std::shared_ptr<Observer<T>>& observer : observers)
    {
        observer->receiveValue(val);
    }
}
