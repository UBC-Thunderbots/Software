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
