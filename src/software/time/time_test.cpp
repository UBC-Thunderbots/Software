#include "software/time/time.h"

#include <gtest/gtest.h>

TEST(TimeTest, test_now)
{
    double t1 = Time::now();
    double t2 = Time::now();
    EXPECT_NE(t1, t2);
}

TEST(TimeTest, test_seconds_since)
{
    const auto start_time = std::chrono::system_clock::now();
    EXPECT_TRUE(Time::secondsSince(start_time) > 0);
}

TEST(TimeTest, test_milliseconds_since)
{
    const auto start_time = std::chrono::system_clock::now();
    EXPECT_TRUE(Time::millisecondsSince(start_time) > 0);
}
