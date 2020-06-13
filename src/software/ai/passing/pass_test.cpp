#include "software/ai/passing/pass.h"

#include <gtest/gtest.h>

TEST(PassTest, constructing_pass_with_negative_speed)
{
    EXPECT_THROW(Pass(Point(1, 2), Point(3, 4), -0.1, Timestamp::fromSeconds(10)),
                 std::invalid_argument);
}

TEST(PassTest, simple_getters)
{
    Pass p(Point(1, 2), Point(3, 4), 3.443, Timestamp::fromSeconds(10));

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    EXPECT_EQ(3.443, p.speed());
    EXPECT_DOUBLE_EQ(std::sqrt(8) / 3.443 + 10, p.estimateReceiveTime().getSeconds());
    EXPECT_DOUBLE_EQ(std::sqrt(8) / 3.443, p.estimatePassDuration().getSeconds());
    EXPECT_EQ(Timestamp::fromSeconds(10), p.startTime());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_x_axis)
{
    Pass p({0, 0}, {-1, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(0, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_y_axis)
{
    Pass p({0, 0}, {0, -1}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(90, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_x_axis)
{
    Pass p({0, 0}, {1, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(180, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_y_axis)
{
    Pass p({0, 0}, {0, 1}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(-90, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_diagonal_to_receiver)
{
    Pass p({1, -1}, {4, 1}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_x_axis)
{
    Pass p({-1, 0}, {0, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(0, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_y_axis)
{
    Pass p({0, -1}, {0, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(90, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_x_axis)
{
    Pass p({1, 0}, {0, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(180, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_y_axis)
{
    Pass p({0, 1}, {0, 0}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(-90, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_passer_diagonal_to_receiver)
{
    Pass p({4, 1}, {1, -1}, 1, Timestamp::fromSeconds(10));
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, stream_operator)
{
    Pass p(Point(1, 2), Point(3, 4), 99.97, Timestamp::fromSeconds(10));

    std::stringstream out;
    out << p;
    EXPECT_EQ("Receiver: (3, 4), Passer: (1, 2) Speed (m/s): 99.97 Start Time (s): 10",
              out.str());
}
