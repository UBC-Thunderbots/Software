#include "multithreading/thread_safe_buffer.h"

#include <gtest/gtest.h>
#include <thread>

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_single_value_when_value_already_on_buffer_length_one){
    ThreadSafeBuffer<int> buffer(1);

    buffer.push(7);

    EXPECT_EQ(7, buffer.pullLeastRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_single_value_when_value_already_on_buffer_length_greater_then_one){
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);

    EXPECT_EQ(7, buffer.pullLeastRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_multiple_value_when_value_already_on_buffer_length_greater_then_one){ ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);
    buffer.push(8);
    buffer.push(9);

    EXPECT_EQ(7, buffer.pullLeastRecentlyAddedValue());
    EXPECT_EQ(8, buffer.pullLeastRecentlyAddedValue());
    EXPECT_EQ(9, buffer.pullLeastRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullLeastRecentlyAddedValue_single_value_when_buffer_is_empty){       ThreadSafeBuffer<int> buffer(3);

    int result = 0;

    // This "pullLeastRecentlyAddedValue" call should block until something is "pushed"
    std::thread puller_thread([&](){ result = buffer.pullLeastRecentlyAddedValue();});

    buffer.push(84);

    // Wait for the pullLeastRecentlyAddedValue to complete
    puller_thread.join();

    EXPECT_EQ(84, result);
}

TEST(ThreadSafeBufferTest, pullMostRecentlyAddedValue_single_value_when_value_already_on_buffer_length_one){
    ThreadSafeBuffer<int> buffer(1);

    buffer.push(7);

    EXPECT_EQ(7, buffer.pullMostRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullMostRecentlyAddedValue_single_value_when_value_already_on_buffer_length_greater_then_one){
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);

    EXPECT_EQ(7, buffer.pullMostRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullMostRecentlyAddedValue_multiple_value_when_value_already_on_buffer_length_greater_then_one){ ThreadSafeBuffer<int> buffer(3);

    buffer.push(7);
    buffer.push(8);
    buffer.push(9);

    EXPECT_EQ(9, buffer.pullMostRecentlyAddedValue());
    EXPECT_EQ(8, buffer.pullMostRecentlyAddedValue());
    EXPECT_EQ(7, buffer.pullMostRecentlyAddedValue());
}

TEST(ThreadSafeBufferTest, pullMostRecentlyAddedValue_single_value_when_buffer_is_empty){       ThreadSafeBuffer<int> buffer(3);

    int result = 0;

    // This "pullMostRecentlyAddedValue" call should block until something is "pushed"
    std::thread puller_thread([&](){ result = buffer.pullMostRecentlyAddedValue();});

    buffer.push(84);

    // Wait for the pullMostRecentlyAddedValue to complete
    puller_thread.join();

    EXPECT_EQ(84, result);
}

TEST(ThreadSafeBufferTest, push_more_values_then_buffer_can_hold){
    ThreadSafeBuffer<int> buffer(3);

    buffer.push(37);
    buffer.push(38);
    buffer.push(39);
    buffer.push(40);

    // We should have overwritten the least recently added value
    EXPECT_EQ(38, buffer.pullLeastRecentlyAddedValue());
    EXPECT_EQ(39, buffer.pullLeastRecentlyAddedValue());
    EXPECT_EQ(40, buffer.pullLeastRecentlyAddedValue());
}

// TODO: test where we destruct a thread safe buffer once
// TODO: test where we destruct a thread safe buffer multiple times
// TODO: test where we put in and pull out an object that can't be copied, like a `unique_ptr`

