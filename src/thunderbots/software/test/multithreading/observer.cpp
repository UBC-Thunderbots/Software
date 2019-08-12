#include "multithreading/observer.h"

#include <gtest/gtest.h>

#include <thread>

class TestObserver : public Observer<int> {
public:
    int getMostRecentValueFromBufferWrapper(){
        return getMostRecentValueFromBuffer();
    }
};

TEST(Observer,
     receiveValue_value_already_available)
{
    TestObserver test_observer;

    test_observer.receiveValue(202);

    EXPECT_EQ(202, test_observer.getMostRecentValueFromBufferWrapper());
}

TEST(Observer,
     receiveValue_value_not_yet_available)
{
    TestObserver test_observer;

    // Create a seperate thread to grab the value for us
    int actual = 0;
    std::thread receive_value_thread([&](){
        actual = test_observer.getMostRecentValueFromBufferWrapper();
    });

    // Send the value over
    test_observer.receiveValue(202);

    // Wait for the thread to successfully get the value
    receive_value_thread.join();

    EXPECT_EQ(202, actual);
}
