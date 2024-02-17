#include "software/ai/passing/pass.h"

#include <gtest/gtest.h>

TEST(PassTest, constructing_pass_with_negative_speed)
{
    EXPECT_THROW(Pass(Point(1, 2), Point(3, 4), -0.1), std::invalid_argument);
}

TEST(PassTest, simple_getters)
{
    Pass p(Point(1, 2), Point(3, 4), 3.443);

    EXPECT_EQ(Point(1, 2), p.passerPoint());
    EXPECT_EQ(Point(3, 4), p.receiverPoint());
    EXPECT_EQ(3.443, p.speed());
    EXPECT_DOUBLE_EQ(std::sqrt(8) / 3.443, p.estimatePassDuration().toSeconds());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_x_axis)
{
    Pass p({0, 0}, {-1, 0}, 1);
    EXPECT_DOUBLE_EQ(0, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_neg_y_axis)
{
    Pass p({0, 0}, {0, -1}, 1);
    EXPECT_DOUBLE_EQ(90, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_x_axis)
{
    Pass p({0, 0}, {1, 0}, 1);
    EXPECT_DOUBLE_EQ(180, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_at_center_and_receiver_on_pos_y_axis)
{
    Pass p({0, 0}, {0, 1}, 1);
    EXPECT_DOUBLE_EQ(-90, p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, receiverOrientation_passer_diagonal_to_receiver)
{
    Pass p({1, -1}, {4, 1}, 1);
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.receiverOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_x_axis)
{
    Pass p({-1, 0}, {0, 0}, 1);
    EXPECT_DOUBLE_EQ(0, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_neg_y_axis)
{
    Pass p({0, -1}, {0, 0}, 1);
    EXPECT_DOUBLE_EQ(90, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_x_axis)
{
    Pass p({1, 0}, {0, 0}, 1);
    EXPECT_DOUBLE_EQ(180, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_receiver_at_center_and_passer_on_pos_y_axis)
{
    Pass p({0, 1}, {0, 0}, 1);
    EXPECT_DOUBLE_EQ(-90, p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, passerOrientation_passer_diagonal_to_receiver)
{
    Pass p({4, 1}, {1, -1}, 1);
    EXPECT_DOUBLE_EQ(-180 + atan(2.0 / 3.0) * 180 / M_PI,
                     p.passerOrientation().mod(Angle::full()).toDegrees());
}

TEST(PassTest, fromPassArray_toPassArray_test)
{
    const std::array<double, 2> pass_array = {1, 1};
    Pass test                        = Pass::fromPassArray(Point(0, 0), pass_array, 2.0);

    EXPECT_EQ(test.toPassArray(), pass_array);
}
