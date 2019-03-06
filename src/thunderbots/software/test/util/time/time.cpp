#include "util/time/time.h"

#include <gtest/gtest.h>

#include <stdexcept>

TEST(TimeTest, create_time_from_positive_seconds)
{
    Time time = Time::fromSeconds(1.0);
    EXPECT_DOUBLE_EQ(time.getSeconds(), 1.0);
}

TEST(TimeTest, create_time_from_negative_seconds)
{
    Time time = Time::fromSeconds(-0.2);
    EXPECT_DOUBLE_EQ(time.getSeconds(), -0.2);
}

TEST(TimeTest, create_time_from_zero_milliseconds)
{
    Time time = Time::fromMilliseconds(0);
    EXPECT_DOUBLE_EQ(time.getMilliseconds(), 0);
}

TEST(TimeTest, create_time_from_negative_milliseconds)
{
    Time time = Time::fromMilliseconds(-125);
    EXPECT_DOUBLE_EQ(time.getMilliseconds(), -125);
}

TEST(TimeTest, create_time_from_positive_milliseconds)
{
    Time time = Time::fromMilliseconds(125);
    EXPECT_DOUBLE_EQ(time.getMilliseconds(), 125);
}

TEST(TimeTest, get_seconds_time_in_milliseconds)
{
    Time time = Time::fromSeconds(5);
    EXPECT_DOUBLE_EQ(time.getMilliseconds(), 5000);
}

TEST(TimeTest, create_millisecond_time_in_seconds)
{
    Time time = Time::fromMilliseconds(300);
    EXPECT_DOUBLE_EQ(time.getSeconds(), 0.3);
}

TEST(TimeTest, test_equality_operator)
{
    Time t1 = Time::fromSeconds(6.2);
    Time t2 = Time::fromMilliseconds(6200);
    EXPECT_TRUE(t1 == t2);
}

TEST(TimeTest, test_inequality_operator)
{
    Time t1 = Time::fromSeconds(0);
    Time t2 = Time::fromMilliseconds(6.2);
    EXPECT_TRUE(t1 != t2);
    EXPECT_FALSE(t2 != t2);
}

TEST(TimeTest, test_less_than_operator)
{
    Time t1 = Time::fromSeconds(6.15);
    Time t2 = Time::fromSeconds(6.2);
    EXPECT_TRUE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
}

TEST(TimeTest, test_less_than_or_equal_to_operator)
{
    Time t1 = Time::fromSeconds(6.2);
    Time t2 = Time::fromSeconds(6.2);
    Time t3 = Time::fromSeconds(0.88);
    EXPECT_TRUE(t3 <= t2);
    EXPECT_TRUE(t2 <= t1);
    EXPECT_FALSE(t1 <= t3);
}

TEST(TimeTest, test_greater_than_operator)
{
    Time t1 = Time::fromSeconds(0.4);
    Time t2 = Time::fromSeconds(3.01);
    EXPECT_TRUE(t2 > t1);
    EXPECT_FALSE(t1 > t2);
}

TEST(TimeTest, test_greater_than_or_equal_to_operator)
{
    Time t1 = Time::fromSeconds(0.4);
    Time t2 = Time::fromSeconds(3.01);
    EXPECT_TRUE(t2 >= t1);
    EXPECT_TRUE(t2 >= t2);
    EXPECT_FALSE(t1 >= t2);
}

TEST(TimeTest, test_addition_operator)
{
    Time t1              = Time::fromSeconds(0.12);
    Time t2              = Time::fromMilliseconds(350);
    Time result          = t1 + t2;
    Time expected_result = Time::fromMilliseconds(470);
    EXPECT_EQ(result, expected_result);
}

TEST(TimeTest, test_subtraction_operator_with_positive_result)
{
    Time t1              = Time::fromMilliseconds(1234);
    Time t2              = Time::fromMilliseconds(600);
    Time result          = t1 - t2;
    Time expected_result = Time::fromMilliseconds(634);
    EXPECT_EQ(result, expected_result);
}

TEST(TimeTest, test_subtraction_operator_with_negative_result)
{
    Time t1              = Time::fromMilliseconds(1234);
    Time t2              = Time::fromMilliseconds(1235);
    Time result          = t1 - t2;
    Time expected_result = Time::fromMilliseconds(-1);
    EXPECT_EQ(result, expected_result);
}
