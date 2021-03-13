#include "software/ai/passing/pass.h"

#include <gtest/gtest.h>

TEST(PassTest, constructing_pass_with_negative_speed)
{
    EXPECT_THROW(Pass(Point(3, 4), -0.1), std::invalid_argument);
}

TEST(PassTest, simple_getters)
{
    Pass p(Point(3, 4), 3.443);

    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    EXPECT_EQ(3.443, p.speed());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_x_axis)
{
    Pass p({-1, 0}, 1);
    EXPECT_DOUBLE_EQ(0, p.receiverOrientation({0, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_y_axis)
{
    Pass p({0, -1}, 1);
    EXPECT_DOUBLE_EQ(90, p.receiverOrientation({0, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_x_axis)
{
    Pass p({1, 0}, 1);
    EXPECT_DOUBLE_EQ(180, p.receiverOrientation({0, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_y_axis)
{
    Pass p({0, 1}, 1);
    EXPECT_DOUBLE_EQ(-90, p.receiverOrientation({0, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_diagonal_to_receiver)
{
    Pass p({4, 1}, 1);
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.receiverOrientation({1, -1}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_x_axis)
{
    Pass p({0, 0}, 1);
    EXPECT_DOUBLE_EQ(0, p.passerOrientation({-1, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_y_axis)
{
    Pass p({0, 0}, 1);
    EXPECT_DOUBLE_EQ(90, p.passerOrientation({0, -1}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_x_axis)
{
    Pass p({0, 0}, 1);
    EXPECT_DOUBLE_EQ(180, p.passerOrientation({1, 0}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_y_axis)
{
    Pass p({0, 0}, 1);
    EXPECT_DOUBLE_EQ(-90, p.passerOrientation({0, 1}).mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_passer_diagonal_to_receiver)
{
    Pass p({1, -1}, 1);
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.passerOrientation({4, 1}).mod(Angle::full()).toDegrees());
}
