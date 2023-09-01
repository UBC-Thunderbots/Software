#include "software/time/duration.h"

#include <gtest/gtest.h>

TEST(DurationTest, default_constructor)
{
    Duration d;
    EXPECT_DOUBLE_EQ(d.toSeconds(), 0);
}

TEST(DurationTest, create_duration_from_positive_seconds)
{
    Duration duration = Duration::fromSeconds(1.0);
    EXPECT_DOUBLE_EQ(duration.toSeconds(), 1.0);
}

TEST(DurationTest, create_duration_from_negative_seconds)
{
    Duration duration = Duration::fromSeconds(-0.2);
    EXPECT_DOUBLE_EQ(duration.toSeconds(), -0.2);
}

TEST(DurationTest, create_duration_from_zero_milliseconds)
{
    Duration duration = Duration::fromMilliseconds(0);
    EXPECT_DOUBLE_EQ(duration.toMilliseconds(), 0);
}

TEST(DurationTest, create_duration_from_negative_milliseconds)
{
    Duration duration = Duration::fromMilliseconds(-125);
    EXPECT_DOUBLE_EQ(duration.toMilliseconds(), -125);
}

TEST(DurationTest, create_duration_from_positive_milliseconds)
{
    Duration duration = Duration::fromMilliseconds(125);
    EXPECT_DOUBLE_EQ(duration.toMilliseconds(), 125);
}

TEST(DurationTest, get_seconds_duration_in_milliseconds)
{
    Duration duration = Duration::fromSeconds(5);
    EXPECT_DOUBLE_EQ(duration.toMilliseconds(), 5000);
}

TEST(DurationTest, get_seconds_duration_in_seconds)
{
    Duration duration = Duration::fromSeconds(5);
    EXPECT_DOUBLE_EQ(duration.toSeconds(), 5);
}

TEST(DurationTest, create_millisecond_duration_in_seconds)
{
    Duration duration = Duration::fromMilliseconds(300);
    EXPECT_DOUBLE_EQ(duration.toSeconds(), 0.3);
}

TEST(DurationTest, test_equality_operator)
{
    Duration d1 = Duration::fromSeconds(6.2);
    Duration d2 = Duration::fromMilliseconds(6200);
    EXPECT_TRUE(d1 == d2);
}

TEST(DurationTest, test_inequality_operator)
{
    Duration d1 = Duration::fromSeconds(0);
    Duration d2 = Duration::fromMilliseconds(6.2);
    EXPECT_TRUE(d1 != d2);
    EXPECT_FALSE(d2 != d2);
}

TEST(DurationTest, test_less_than_operator)
{
    Duration d1 = Duration::fromSeconds(6.15);
    Duration d2 = Duration::fromSeconds(6.2);
    EXPECT_TRUE(d1 < d2);
    EXPECT_FALSE(d2 < d1);
}

TEST(DurationTest, test_less_than_or_equal_to_operator)
{
    Duration d1 = Duration::fromSeconds(6.2);
    Duration d2 = Duration::fromSeconds(6.2);
    Duration d3 = Duration::fromSeconds(0.88);
    EXPECT_TRUE(d3 <= d2);
    EXPECT_TRUE(d2 <= d1);
    EXPECT_FALSE(d1 <= d3);
}

TEST(DurationTest, test_greater_than_operator)
{
    Duration d1 = Duration::fromSeconds(0.4);
    Duration d2 = Duration::fromSeconds(3.01);
    EXPECT_TRUE(d2 > d1);
    EXPECT_FALSE(d1 > d2);
}

TEST(DurationTest, test_greater_than_or_equal_to_operator)
{
    Duration d1 = Duration::fromSeconds(0.4);
    Duration d2 = Duration::fromSeconds(3.01);
    EXPECT_TRUE(d2 >= d1);
    EXPECT_TRUE(d2 >= d2);
    EXPECT_FALSE(d1 >= d2);
}
TEST(DurationTest, test_addition_operator)
{
    Duration d1              = Duration::fromSeconds(0.12);
    Duration d2              = Duration::fromMilliseconds(350);
    Duration result          = d1 + d2;
    Duration expected_result = Duration::fromMilliseconds(470);
    EXPECT_EQ(result, expected_result);
}

TEST(DurationTest, test_subtraction_operator_with_positive_result)
{
    Duration d1              = Duration::fromMilliseconds(1234);
    Duration d2              = Duration::fromMilliseconds(600);
    Duration result          = d1 - d2;
    Duration expected_result = Duration::fromMilliseconds(634);
    EXPECT_EQ(result, expected_result);
}

TEST(DurationTest, test_subtraction_operator_with_negative_result)
{
    Duration d1              = Duration::fromMilliseconds(1234);
    Duration d2              = Duration::fromMilliseconds(1235);
    Duration result          = d1 - d2;
    Duration expected_result = Duration::fromMilliseconds(-1);
    EXPECT_EQ(result, expected_result);
}

TEST(DurationTest, test_addition_assignment_operator)
{
    Duration d              = Duration::fromSeconds(0.12);
    d += Duration::fromMilliseconds(350);
    Duration expected_result = Duration::fromMilliseconds(470);
    EXPECT_EQ(d, expected_result);
}

TEST(DurationTest, test_subtraction_assignment_operator_with_positive_result)
{
    Duration d              = Duration::fromMilliseconds(1234);
    d -= Duration::fromMilliseconds(600);
    Duration expected_result = Duration::fromMilliseconds(634);
    EXPECT_EQ(d, expected_result);
}

TEST(DurationTest, test_subtraction_assignment_operator_with_negative_result)
{
    Duration d              = Duration::fromMilliseconds(1234);
    d -= Duration::fromMilliseconds(1235);
    Duration expected_result = Duration::fromMilliseconds(-1);
    EXPECT_EQ(d, expected_result);
}

TEST(DurationTest, stream_operator)
{
    Duration d = Duration::fromSeconds(3.0);

    std::stringstream out;
    out << d;

    EXPECT_EQ("3.00s", out.str());
}
