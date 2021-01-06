#pragma once

template <typename T>
class Observer {
   public:
    virtual void receiveValue(T val) = 0;
};