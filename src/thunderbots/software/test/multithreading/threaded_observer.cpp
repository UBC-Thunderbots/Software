#include "multithreading/threaded_observer.h"

#include <gtest/gtest.h>

#include <thread>

using namespace std::chrono_literals;

class TestThreadedObserver : public ThreadedObserver<int> {
public:
    int received_value;
    void onValueReceived(int i){
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
    auto test_threaded_observer = std::make_shared<TestThreadedObserver>();

    test_threaded_observer->receiveValue(10);
    test_threaded_observer->receiveValue(20);

    test_threaded_observer.reset();
}

//TEST(Observer,
//     receiveValue_value_already_available)
//{
//    TestObserver test_observer;
//
//    test_observer.receiveValue(202);
//
//    EXPECT_EQ(202, test_observer.getMostRecentValueFromBufferWrapper());
//}
//
//TEST(Observer,
//     receiveValue_value_not_yet_available)
//{
//    TestObserver test_observer;
//
//    // Create a seperate thread to grab the value for us
//    int actual = 0;
//    std::thread receive_value_thread([&](){
//        actual = test_observer.getMostRecentValueFromBufferWrapper();
//    });
//
//    // Send the value over
//    test_observer.receiveValue(202);
//
//    // Wait for the thread to successfully get the value
//    receive_value_thread.join();
//
//    EXPECT_EQ(202, actual);
//}
