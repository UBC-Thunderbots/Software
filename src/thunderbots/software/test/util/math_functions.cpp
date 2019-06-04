//
// Created by roark on 28/03/19.
//

#include "util/math_functions.h"

#include <gtest/gtest.h>

TEST(LinearUtilFunctionTest, testZeroCase)
{
    double out = Util::linear(0, 0, 2);
    EXPECT_EQ(out, 0.5);
}

TEST(LinearUtilFunctionTest, testOneQuarter)
{
    double out = Util::linear(-1, 0, 4);
    EXPECT_EQ(out, 0.25);
}

TEST(LinearUtilFunctionTest, testTwoThirds)
{
    double out = Util::linear(0.75, 0, 4.5);
    EXPECT_EQ(out, 2.0 / 3.0);
}

TEST(LinearUtilFunctionTest, testMinimumNoOffset)
{
    double out = Util::linear(-1.5, 0, 3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNoOffset)
{
    double out = Util::linear(2.5, 0, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNoOffset)
{
    double out = Util::linear(-2, 0, 1);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNoOffset)
{
    double out = Util::linear(4.2, 0, 6);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumNegativeOffset)
{
    double out = Util::linear(-4, -2, 4);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNegativeOffset)
{
    double out = Util::linear(1.5, -1, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumPositiveOffset)
{
    double out = Util::linear(0, 3, 6);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumPositiveOffset)
{
    double out = Util::linear(6, 1.5, 9);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNegativeOffset)
{
    double out = Util::linear(-1.8, -0.2, 0.7);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNegativeOffset)
{
    double out = Util::linear(0.18, -0.05, 0.24);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowPositiveOffset)
{
    double out = Util::linear(-0.3, 0.6, 1.3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAbovePositiveOffset)
{
    double out = Util::linear(0.35, 0.15, 0.18);
    EXPECT_EQ(out, 1.0);
}

TEST(SigmoidTest, rectangleSigmoid_value_in_rectangle_centered_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {0, 0}, 0.1), 0.982);
}

TEST(SigmoidTest, rectangleSigmoid_value_in_rectangle_offset_rectangle)
{
    Rectangle r1(Point(-2, -1), Point(0, 3));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {-1, 1}, 0.1), 0.9);
}

TEST(SigmoidTest, rectangleSigmoid_values_outside_rectangle)
{
    Rectangle r1(Point(-1, -2), Point(1, 2));

    // Check that values off in x are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.2, 0}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.2, 0}, 0.1), 0.1);

    // Check that values off in y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {0, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {0, 2.1}, 0.1), 0.1);

    // Check that values off in x and y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, -2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {-1.1, 2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1, 2.1}, 0.1), 0.1);
}

TEST(SigmoidTest, circleSigmoid_value_in_circle_centered_circle)
{
    Circle circle(Point(0, 0), 1);

    EXPECT_GE(circleSigmoid(circle, {0, 0}, 0.1), 0.982);
}

TEST(SigmoidTest, circleSigmoid_value_in_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_GE(circleSigmoid(circle, {1, -1}, 0.1), 0.982);
}

TEST(SigmoidTest, circleSigmoid_value_on_circle_edge)
{
    Circle circle(Point(1, -1), 1);

    EXPECT_EQ(circleSigmoid(circle, {0, -1}, 0.1), 0.5);
    EXPECT_NEAR(circleSigmoid(circle, {1 + std::sqrt(2) / 2, -1 - std::sqrt(2) / 2}, 0.1),
                0.5, 0.01);
}

TEST(SigmoidTest, circleSigmoid_value_outside_circle_offset_circle)
{
    Circle circle(Point(1, -1), 1);

    // Check that values off in x are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, 0}, 0.1), 0.018);

    // Check that values off in y are basically 0
    EXPECT_LE(circleSigmoid(circle, {0, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0, -2.2}, 0.1), 0.018);

    // Check that values off in x and y are basically 0
    EXPECT_LE(circleSigmoid(circle, {-0.2, 0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {2.2, -1.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {0.2, -0.2}, 0.1), 0.018);
    EXPECT_LE(circleSigmoid(circle, {-2.2, 1.2}, 0.1), 0.018);
}

TEST(SigmoidTest, sigmoid_sig_width_is_respected)
{
    // Test the value at sig_width/2 is 0.982
    EXPECT_NEAR(sigmoid(5, 0, 10), 0.982, 0.0001);

    // Test the value at -sig_width/2 is 0.018
    EXPECT_NEAR(sigmoid(-5, 0, 10), 0.018, 0.001);
}

TEST(SigmoidTest, sigmoid_offset)
{
    // Test that the value at 0 is 0.5 if no offset
    EXPECT_DOUBLE_EQ(0.5, sigmoid(0, 0, 1));

    // Test that the value at the offset is 0.5
    EXPECT_DOUBLE_EQ(0.5, sigmoid(2, 2, 1));
    EXPECT_DOUBLE_EQ(0.5, sigmoid(-2, -2, 1));
}

TEST(SigmoidTest, sigmoid_negating_sig_width_flips_sigmoid)
{
    // Test that negating sig_width inverts the sigmoid
    EXPECT_NEAR(sigmoid(-5, 0, -10), 0.982, 0.0001);
    EXPECT_NEAR(sigmoid(5, 0, -10), 0.018, 0.0001);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
