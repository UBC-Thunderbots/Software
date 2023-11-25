#include "software/time/timestamp.h"

#include <gtest/gtest.h>

TEST(TimestampTest, default_constructor)
{
    Timestamp t;
    EXPECT_DOUBLE_EQ(t.toSeconds(), 0);
}

TEST(TimestampTest, create_timestamp_from_positive_timestamp_protobuf)
{
    TbotsProto::Timestamp timestamp_msg;
    timestamp_msg.set_epoch_timestamp_seconds(1.0);
    Timestamp t = Timestamp::fromTimestampProto(timestamp_msg);
    EXPECT_DOUBLE_EQ(t.toSeconds(), 1.0);
}

TEST(TimestampTest, create_timestamp_from_negative_timestamp_protobuf)
{
    TbotsProto::Timestamp timestamp_msg;
    timestamp_msg.set_epoch_timestamp_seconds(-1.0);
    EXPECT_THROW(Timestamp::fromTimestampProto(timestamp_msg), std::invalid_argument);
}

TEST(TimestampTest, create_timestamp_from_positive_seconds)
{
    Timestamp timestamp = Timestamp::fromSeconds(1.0);
    EXPECT_DOUBLE_EQ(timestamp.toSeconds(), 1.0);
}

TEST(TimestampTest, create_timestamp_from_negative_seconds)
{
    EXPECT_THROW(Timestamp::fromSeconds(-0.2), std::invalid_argument);
}

TEST(TimestampTest, create_timestamp_from_zero_milliseconds)
{
    Timestamp timestamp = Timestamp::fromMilliseconds(0);
    EXPECT_DOUBLE_EQ(timestamp.toMilliseconds(), 0);
}

TEST(TimestampTest, create_timestamp_from_negative_milliseconds)
{
    EXPECT_THROW(Timestamp::fromMilliseconds(-125), std::invalid_argument);
}

TEST(TimestampTest, get_seconds_timestamp_in_milliseconds)
{
    Timestamp timestamp = Timestamp::fromSeconds(5);
    EXPECT_DOUBLE_EQ(timestamp.toMilliseconds(), 5000);
}

TEST(TimestampTest, create_millisecond_timestamp_in_seconds)
{
    Timestamp timestamp = Timestamp::fromMilliseconds(300);
    EXPECT_DOUBLE_EQ(timestamp.toSeconds(), 0.3);
}

TEST(TimestampTest, test_equality_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(6.2);
    Timestamp t2 = Timestamp::fromMilliseconds(6200);
    EXPECT_TRUE(t1 == t2);
}

TEST(TimestampTest, test_inequality_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(0);
    Timestamp t2 = Timestamp::fromMilliseconds(6.2);
    EXPECT_TRUE(t1 != t2);
    EXPECT_FALSE(t2 != t2);
}

TEST(TimestampTest, test_less_than_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(6.15);
    Timestamp t2 = Timestamp::fromSeconds(6.2);
    EXPECT_TRUE(t1 < t2);
    EXPECT_FALSE(t2 < t1);
}

TEST(TimestampTest, test_less_than_or_equal_to_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(6.2);
    Timestamp t2 = Timestamp::fromSeconds(6.2);
    Timestamp t3 = Timestamp::fromSeconds(0.88);
    EXPECT_TRUE(t3 <= t2);
    EXPECT_TRUE(t2 <= t1);
    EXPECT_FALSE(t1 <= t3);
}

TEST(TimestampTest, test_greater_than_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(0.4);
    Timestamp t2 = Timestamp::fromSeconds(3.01);
    EXPECT_TRUE(t2 > t1);
    EXPECT_FALSE(t1 > t2);
}

TEST(TimestampTest, test_greater_than_or_equal_to_operator)
{
    Timestamp t1 = Timestamp::fromSeconds(0.4);
    Timestamp t2 = Timestamp::fromSeconds(3.01);
    EXPECT_TRUE(t2 >= t1);
    EXPECT_TRUE(t2 >= t2);
    EXPECT_FALSE(t1 >= t2);
}

TEST(TimestampTest, test_addition_operator_with_duration)
{
    Timestamp t1              = Timestamp::fromSeconds(0.12);
    Duration d1               = Duration::fromMilliseconds(350);
    Timestamp result          = t1 + d1;
    Timestamp expected_result = Timestamp::fromMilliseconds(470);
    EXPECT_EQ(result, expected_result);
}

TEST(TimestampTest, test_subtraction_operator_with_duration_with_positive_result)
{
    Timestamp t1              = Timestamp::fromMilliseconds(1234);
    Duration d1               = Duration::fromMilliseconds(600);
    Timestamp result          = t1 - d1;
    Timestamp expected_result = Timestamp::fromMilliseconds(634);
    EXPECT_EQ(result, expected_result);
}

TEST(TimestampTest, test_subtraction_operator_with_duration_with_negative_result)
{
    Timestamp t1 = Timestamp::fromMilliseconds(1234);
    Duration d1  = Duration::fromMilliseconds(1235);
    EXPECT_THROW(t1 - d1, std::invalid_argument);
}

TEST(TimestampTest, test_subtraction_operator_with_timestamp_with_negative_result)
{
    Timestamp t1             = Timestamp::fromMilliseconds(1234);
    Timestamp t2             = Timestamp::fromMilliseconds(600);
    Duration result          = t1 - t2;
    Duration expected_result = Duration::fromMilliseconds(634);
    EXPECT_EQ(result, expected_result);
}

TEST(TimestampTest, stream_operator)
{
    Timestamp t = Timestamp::fromSeconds(2.00);

    std::stringstream out;
    out << t;

    EXPECT_EQ("2.00s", out.str());
}
