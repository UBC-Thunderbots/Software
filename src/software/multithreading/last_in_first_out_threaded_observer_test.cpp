#include "software/multithreading/last_in_first_out_threaded_observer.h"

#include <gtest/gtest.h>

#include <thread>

using namespace std::chrono_literals;

class TestThreadedObserver : public LastInFirstOutThreadedObserver<int>
{
   public:
    int received_value = 0;

   private:
    void onValueReceived(int i) override
    {
        received_value = i;
    }
};

class TestVectorThreadedObserver : public LastInFirstOutThreadedObserver<int>
{
   public:
    TestVectorThreadedObserver() : LastInFirstOutThreadedObserver(10) {}

    std::vector<int> received_values;

   private:
    void onValueReceived(int i) override
    {
        received_values.emplace_back(i);
        // sleep for a while to ensure that the buffer fills up
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
};

TEST(LastInFirstOutThreadedObserver, receiveValue)
{
    TestThreadedObserver test_threaded_observer;

    test_threaded_observer.receiveValue(83);

    std::this_thread::sleep_for(5s);

    // The value should have been updated by the thread running
    // in the ThreadedObserver
    EXPECT_EQ(83, test_threaded_observer.received_value);
}

TEST(LastInFirstOutThreadedObserver, receiveMultipleValuesInOrder)
{
    TestVectorThreadedObserver test_vector_threaded_observer;
    std::vector<int> test_values{1, 2, 3, 4, 5};
    std::vector<int> expected_values(test_values.rbegin(), test_values.rend());

    for (auto num : test_values)
    {
        test_vector_threaded_observer.receiveValue(num);
    }

    std::this_thread::sleep_for(5s);

    EXPECT_EQ(test_vector_threaded_observer.received_values, expected_values);
}

TEST(LastInFirstOutThreadedObserver, destructor)
{
    // Because the destructor has to manage the internal thread to make sure it
    // finishes, this test ensures that it actually can succeed

    auto test_threaded_observer = std::make_shared<TestThreadedObserver>();

    test_threaded_observer->receiveValue(10);
    test_threaded_observer->receiveValue(20);

    test_threaded_observer.reset();
}
