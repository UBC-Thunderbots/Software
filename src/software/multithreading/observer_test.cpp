#include "software/multithreading/observer.h"

#include <gtest/gtest.h>

#include <thread>

class TestObserver : public Observer<int>
{
   public:
    std::optional<int> getMostRecentValueFromBufferWrapper()
    {
        return popMostRecentlyReceivedValue(Duration::fromSeconds(5));
    }
};

TEST(Observer, receiveValue_value_already_available)
{
    TestObserver test_observer;

    test_observer.receiveValue(202);

    std::optional<int> result = test_observer.getMostRecentValueFromBufferWrapper();
    ASSERT_TRUE(result);
    EXPECT_EQ(202, *result);
}

TEST(Observer, receiveValue_value_not_yet_available)
{
    TestObserver test_observer;

    // Create a separate thread to grab the value for us
    std::optional<int> result = std::nullopt;
    std::thread receive_value_thread([&]() {
        while (!result)
        {
            result = test_observer.getMostRecentValueFromBufferWrapper();
        }
    });

    // Send the value over
    test_observer.receiveValue(202);

    // Wait for the thread to successfully get the value
    receive_value_thread.join();

    ASSERT_TRUE(result);
    EXPECT_EQ(202, *result);
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_filled)
{
    TestObserver test_observer;
    for (unsigned int i = 0; i < TestObserver::TIME_BUFFER_SIZE; i++)
    {
        test_observer.receiveValue(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_NEAR(test_observer.getDataReceivedPerSecond(), 1 / 0.01, 20);
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_filled_twice_over)
{
    TestObserver test_observer;
    for (unsigned int i = 0; i < TestObserver::TIME_BUFFER_SIZE * 2; i++)
    {
        test_observer.receiveValue(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_NEAR(test_observer.getDataReceivedPerSecond(), 1 / 0.005, 20);
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_empty)
{
    TestObserver test_observer;
    EXPECT_EQ(test_observer.getDataReceivedPerSecond(), 0);
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_partially_empty)
{
    TestObserver test_observer;
    for (unsigned int i = 0; i < TestObserver::TIME_BUFFER_SIZE / 2; i++)
    {
        test_observer.receiveValue(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // the data received value is noisy when the buffer is partially empty, so we allow a
    // larger margin of error
    EXPECT_NEAR(test_observer.getDataReceivedPerSecond(), 1 / 0.01, 60);
}
