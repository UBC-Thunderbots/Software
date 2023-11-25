#include "software/multithreading/observer.hpp"

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

namespace TestUtil
{
    /**
     * Tests getDataReceivedPerSecond by filling the buffer
     *
     * @param test_observer The observer to test
     * @param data_received_period_milliseconds The period between receiving data
     * @param number_of_messages number of messages to send to the buffer
     *
     * @return AssertionSuccess the observer returns the correct data received per second
     */
    ::testing::AssertionResult testGetDataReceivedPerSecondByFillingBuffer(
        TestObserver test_observer, unsigned int data_received_period_milliseconds,
        unsigned int number_of_messages)
    {
        auto wall_time_start = std::chrono::steady_clock::now();
        for (unsigned int i = 0; i < number_of_messages; i++)
        {
            test_observer.receiveValue(i);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(data_received_period_milliseconds));
        }

        auto wall_time_now = std::chrono::steady_clock::now();
        double test_duration_s =
            static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                    wall_time_now - wall_time_start)
                                    .count()) *
            SECONDS_PER_MILLISECOND;
        double scaling_factor =
            test_duration_s / (data_received_period_milliseconds *
                               SECONDS_PER_MILLISECOND * number_of_messages);
        double expected_actual_difference =
            std::abs(test_observer.getDataReceivedPerSecond() -
                     1 / (data_received_period_milliseconds * SECONDS_PER_MILLISECOND) *
                         scaling_factor);
        if (expected_actual_difference < 50)
        {
            return ::testing::AssertionSuccess();
        }
        else
        {
            return ::testing::AssertionFailure()
                   << "The difference between expected and actual data received per seconds was "
                   << expected_actual_difference;
        }
    }
};  // namespace TestUtil

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
    EXPECT_TRUE(TestUtil::testGetDataReceivedPerSecondByFillingBuffer(
        TestObserver(), 10, TestObserver::TIME_BUFFER_SIZE));
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_filled_twice_over)
{
    EXPECT_TRUE(TestUtil::testGetDataReceivedPerSecondByFillingBuffer(
        TestObserver(), 5, TestObserver::TIME_BUFFER_SIZE * 2));
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_empty)
{
    TestObserver test_observer;
    EXPECT_EQ(test_observer.getDataReceivedPerSecond(), 0);
}

TEST(Observer, getDataReceivedPerSecond_time_buffer_partially_empty)
{
    EXPECT_TRUE(TestUtil::testGetDataReceivedPerSecondByFillingBuffer(
        TestObserver(), 10, TestObserver::TIME_BUFFER_SIZE / 2));
}
