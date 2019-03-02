#include "util/time/duration.h"

#include <gtest/gtest.h>

#include <stdexcept>

TEST(DurationTest, create_duration_from_positive_seconds)
{
    Duration duration = Duration::fromSeconds(1.0);
    EXPECT_DOUBLE_EQ(duration.getSeconds(), 1.0);
}

TEST(DurationTest, create_duration_from_negative_seconds)
{
    Duration duration = Duration::fromSeconds(-0.2);
    EXPECT_DOUBLE_EQ(duration.getSeconds(), -0.2);
}

TEST(DurationTest, create_duration_from_zero_milliseconds)
{
    Duration duration = Duration::fromMilliseconds(0);
    EXPECT_DOUBLE_EQ(duration.getMilliseconds(), 0);
}

TEST(DurationTest, create_duration_from_negative_milliseconds)
{
    Duration duration = Duration::fromMilliseconds(-125);
    EXPECT_DOUBLE_EQ(duration.getMilliseconds(), -125);
}

TEST(DurationTest, get_seconds_duration_in_milliseconds)
{
    Duration duration = Duration::fromSeconds(5);
    EXPECT_DOUBLE_EQ(duration.getMilliseconds(), 5000);
}

TEST(DurationTest, create_millisecond_duration_in_seconds)
{
    Duration duration = Duration::fromMilliseconds(300);
    EXPECT_DOUBLE_EQ(duration.getSeconds(), 0.3);
}

TEST(DurationTest, test_equality_operator)
{
    Duration t1 = Duration::fromSeconds(6.2);
    Duration t2 = Duration::fromMilliseconds(6200);
    EXPECT_TRUE(t1 == t2);
}

TEST(DurationTest, test_inequality_operator)
{
    Duration t1 = Duration::fromSeconds(0);
    Duration t2 = Duration::fromMilliseconds(6.2);
    EXPECT_TRUE(t1 != t2);
    EXPECT_FALSE(t2 != t2);
}

TEST(DurationTest, test_less_than_operator)
{
    Duration t1 = Duration::fromSeconds(6.15);
    Duration t2 = Duration::fromSeconds(6.2);
    EXPECT_TRUE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
}

TEST(DurationTest, test_less_than_or_equal_to_operator)
{
    Duration t1 = Duration::fromSeconds(6.2);
    Duration t2 = Duration::fromSeconds(6.2);
    Duration t3 = Duration::fromSeconds(0.88);
    EXPECT_TRUE(t3 <= t2);
    EXPECT_TRUE(t2 <= t1);
    EXPECT_FALSE(t1 <= t3);
}

TEST(DurationTest, test_greater_than_operator)
{
    Duration t1 = Duration::fromSeconds(0.4);
    Duration t2 = Duration::fromSeconds(3.01);
    EXPECT_TRUE(t2 > t1);
    EXPECT_FALSE(t1 > t2);
}

TEST(DurationTest, test_greater_than_or_equal_to_operator)
{
    Duration t1 = Duration::fromSeconds(0.4);
    Duration t2 = Duration::fromSeconds(3.01);
    EXPECT_TRUE(t2 >= t1);
    EXPECT_TRUE(t2 >= t2);
    EXPECT_FALSE(t1 >= t2);
}

TEST(DurationTest, test_addition_operator)
{
    Duration t1              = Duration::fromSeconds(0.12);
    Duration t2 = Duration::fromMilliseconds(350);
    Duration result          = t1 + t2;
    Duration expected_result = Duration::fromMilliseconds(470);
    EXPECT_EQ(result, expected_result);
}

TEST(DurationTest, test_subtraction_operator_with_positive_result)
{
    Duration t1              = Duration::fromMilliseconds(1234);
    Duration t2 = Duration::fromMilliseconds(600);
    Duration result          = t1 - t2;
    Duration expected_result = Duration::fromMilliseconds(634);
    EXPECT_EQ(result, expected_result);
}

TEST(DurationTest, test_subtraction_operator_with_negative_result)
{
    Duration t1              = Duration::fromMilliseconds(1234);
    Duration t2 = Duration::fromMilliseconds(1235);
    Duration result          = t1 - t2;
    Duration expected_result = Duration::fromMilliseconds(-1);
    EXPECT_EQ(result, expected_result);
}
