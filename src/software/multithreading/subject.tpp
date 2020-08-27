#pragma once

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

template <typename T>
double Subject<T>::getAverageDataReceivedPerSecond()
{
    double sum = 0;
    for (std::shared_ptr<Observer<T>>& observer : observers)
    {
        sum += observer->getDataReceivedPerSecond();
    }

    return sum / (double)observers.size();
}