#include "software/multithreading/thread_safe_buffer.hpp"

#include <gtest/gtest.h>

#include <thread>

TEST(ThreadSafeBufferTest,
     pullLeastRecentlyAddedValue_single_value_when_value_already_on_buffer_length_one)
{
    ThreadSafeBuffer<int> buffer(1);

    buffer.push(7);

    EXPECT_EQ(std::optional<int>(7), buffer.popLeastRecentlyAddedValue());
}

TEST(
    ThreadSafeBufferTest,
    pullLeastRecentlyAddedValue_single_value_when_value_already_on_buffer_length_greater_then_one)
{
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);

    EXPECT_EQ(7, buffer.popLeastRecentlyAddedValue());
}

TEST(
    ThreadSafeBufferTest,
    pullLeastRecentlyAddedValue_multiple_value_when_value_already_on_buffer_length_greater_then_one)
{
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);
    buffer.push(8);
    buffer.push(9);

    EXPECT_EQ(7, buffer.popLeastRecentlyAddedValue());
    EXPECT_EQ(8, buffer.popLeastRecentlyAddedValue());
    EXPECT_EQ(9, buffer.popLeastRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_single_value_when_buffer_is_empty)
{
    ThreadSafeBuffer<int> buffer(3);

    std::optional<int> result = std::nullopt;

    // This "popLeastRecentlyAddedValue" call should block until something is "pushed"
    std::thread puller_thread([&]() {
        while (!result)
        {
            result = buffer.popLeastRecentlyAddedValue(Duration::fromSeconds(0.1));
        }
    });

    buffer.push(84);

    // Wait for the popLeastRecentlyAddedValue to complete
    puller_thread.join();

    ASSERT_TRUE(result);
    EXPECT_EQ(84, *result);
}

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_single_value_when_buffer_is_full)
{
    ThreadSafeBuffer<int> buffer(2);
    buffer.push(114);
    buffer.push(115);
    buffer.push(116);

    std::optional<int> result = std::nullopt;

    // This "popLeastRecentlyAddedValue" call should block until something is "pushed"
    std::thread puller_thread([&]() {
        while (!result)
        {
            // should find values already in the buffer
            result = buffer.popLeastRecentlyAddedValue(Duration::fromSeconds(2));
        }
    });

    // this push should be too late to affect the previous call
    std::this_thread::sleep_for(std::chrono::seconds(2));
    buffer.push(117);

    // Wait for the popLeastRecentlyAddedValue to complete
    puller_thread.join();

    ASSERT_TRUE(result);
    EXPECT_EQ(115, *result);
}

TEST(ThreadSafeBufferTest,
     pullMostRecentlyAddedValue_single_value_when_value_already_on_buffer_length_one)
{
    ThreadSafeBuffer<int> buffer(1);

    buffer.push(7);

    EXPECT_EQ(7, buffer.popMostRecentlyAddedValue());
}

TEST(
    ThreadSafeBufferTest,
    pullMostRecentlyAddedValue_single_value_when_value_already_on_buffer_length_greater_then_one)
{
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);

    EXPECT_EQ(7, buffer.popMostRecentlyAddedValue());
}

TEST(
    ThreadSafeBufferTest,
    pullMostRecentlyAddedValue_multiple_value_when_value_already_on_buffer_length_greater_then_one)
{
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);
    buffer.push(8);
    buffer.push(9);

    EXPECT_EQ(9, buffer.popMostRecentlyAddedValue());
    EXPECT_EQ(8, buffer.popMostRecentlyAddedValue());
    EXPECT_EQ(7, buffer.popMostRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullMostRecentlyAddedValue_single_value_when_buffer_is_empty)
{
    ThreadSafeBuffer<int> buffer(3);

    std::optional<int> result = std::nullopt;

    // This "popLeastRecentlyAddedValue" call should block until something is "pushed"
    std::thread puller_thread([&]() {
        while (!result)
        {
            result = buffer.popMostRecentlyAddedValue(Duration::fromSeconds(0.1));
        }
    });

    buffer.push(84);

    // Wait for the popMostRecentlyAddedValue to complete
    puller_thread.join();

    ASSERT_TRUE(result);
    EXPECT_EQ(84, *result);
}

TEST(ThreadSafeBufferTest, push_more_values_then_buffer_can_hold)
{
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(37);
    buffer.push(38);
    buffer.push(39);
    buffer.push(40);

    // We should have overwritten the least recently added value
    EXPECT_EQ(38, buffer.popLeastRecentlyAddedValue());
    EXPECT_EQ(39, buffer.popLeastRecentlyAddedValue());
    EXPECT_EQ(40, buffer.popLeastRecentlyAddedValue());
}
