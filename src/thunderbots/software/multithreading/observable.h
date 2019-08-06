#pragma once

#include "multithreading/observer.h"

// TODO: TEST THIS CLASS
/**
 * This class represents something that can be observed.
 *
 * The general usage should be to pass "Observer<T>" objects into "registerObserver".
 * These "Observer<T>" objects will receive new data from this class when it is
 * available
 *
 * @tparam T The type of object that is being provided to all registered Observers
 */
template <typename T>
class Observable
{
   public:
    // TODO: better name for this function?
    /**
     * Register the given observer with this class to receive new values when
     * they are available
     *
     * @param observer
     */
    void registerObserver(std::shared_ptr<Observer<T>> observer);

    virtual ~Observable() = default;

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
void Observable<T>::registerObserver(std::shared_ptr<Observer<T>> observer)
{
    observers.emplace_back(observer);
}

template <typename T>
void Observable<T>::sendValueToObservers(T val)
{
    for (std::shared_ptr<Observer<T>>& observer : observers)
    {
        observer->receiveValue(val);
    }
}
