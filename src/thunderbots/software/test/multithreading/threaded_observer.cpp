#include "multithreading/threaded_observer.h"

#include <gtest/gtest.h>

#include <thread>

using namespace std::chrono_literals;

class TestThreadedObserver : public ThreadedObserver<int> {
public:
    int received_value = 0;
private:
    void onValueReceived(int i) override {
        received_value = i;
    }
};

TEST(ThreadedObserver, receiveValue){
    TestThreadedObserver test_threaded_observer;

    test_threaded_observer.receiveValue(83);

    std::this_thread::sleep_for(5s);

    // The value should have been updated by the thread running
    // in the ThreadedObserver
    EXPECT_EQ(83, test_threaded_observer.received_value);
}

TEST(ThreadedObserver, destructor){
    // Because the destructor has to manage the internal thread to make sure it
    // finishes, this test ensures that it actually can succeed

    auto test_threaded_observer = std::make_shared<TestThreadedObserver>();

    test_threaded_observer->receiveValue(10);
    test_threaded_observer->receiveValue(20);

    test_threaded_observer.reset();
}
