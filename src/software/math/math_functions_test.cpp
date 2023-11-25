#include "software/math/math_functions.h"

#include <gtest/gtest.h>

TEST(LinearUtilFunctionTest, testZeroCase)
{
    double out = linear(0, 0, 2);
    EXPECT_EQ(out, 0.5);
}

TEST(LinearUtilFunctionTest, testOneQuarter)
{
    double out = linear(-1, 0, 4);
    EXPECT_EQ(out, 0.25);
}

TEST(LinearUtilFunctionTest, testTwoThirds)
{
    double out = linear(0.75, 0, 4.5);
    EXPECT_EQ(out, 2.0 / 3.0);
}

TEST(LinearUtilFunctionTest, testMinimumNoOffset)
{
    double out = linear(-1.5, 0, 3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNoOffset)
{
    double out = linear(2.5, 0, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNoOffset)
{
    double out = linear(-2, 0, 1);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNoOffset)
{
    double out = linear(4.2, 0, 6);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumNegativeOffset)
{
    double out = linear(-4, -2, 4);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumNegativeOffset)
{
    double out = linear(1.5, -1, 5.0);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testMinimumPositiveOffset)
{
    double out = linear(0, 3, 6);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testMaximumPositiveOffset)
{
    double out = linear(6, 1.5, 9);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowNegativeOffset)
{
    double out = linear(-1.8, -0.2, 0.7);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAboveNegativeOffset)
{
    double out = linear(0.18, -0.05, 0.24);
    EXPECT_EQ(out, 1.0);
}

TEST(LinearUtilFunctionTest, testClampBelowPositiveOffset)
{
    double out = linear(-0.3, 0.6, 1.3);
    EXPECT_EQ(out, 0.0);
}

TEST(LinearUtilFunctionTest, testClampAbovePositiveOffset)
{
    double out = linear(0.35, 0.15, 0.18);
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

TEST(NormalizeToRangeTest, test_integral_type_normalize_to_same_range)
{
    int result = normalizeValueToRange<int>(64, 0, 100, 0, 100);
    EXPECT_EQ(64, result);
}

TEST(NormalizeToRangeTest, test_integral_type_normalize_to_different_range)
{
    int result = normalizeValueToRange<int>(64, 0, 100, 0, 1000);
    EXPECT_EQ(640, result);
}

TEST(NormalizeToRangeTest, test_integral_type_initial_value_out_of_range)
{
    int result = normalizeValueToRange<int>(12, 0, 10, 0, 1000);
    EXPECT_EQ(1000, result);
}

TEST(NormalizeToRangeTest, test_floating_point_type_normalize_to_same_range)
{
    float result = normalizeValueToRange<float>(39.05f, 0.0f, 50.0f, 0.0f, 50.0f);
    EXPECT_FLOAT_EQ(39.05f, result);
}

TEST(NormalizeToRangeTest, test_floating_point_type_normalize_to_different_range)
{
    float result = normalizeValueToRange<float>(39.05f, 0.0f, 50.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(0.781f, result);
}

TEST(NormalizeToRangeTest, test_floating_point_type_initial_value_out_of_range)
{
    double result = normalizeValueToRange<double>(300, 0, 255, 0, 6);
    EXPECT_DOUBLE_EQ(6.0, result);
}

TEST(PercentDifferenceTest, test_both_numbers_zero)
{
    double result = percent_difference(0, 0);
    EXPECT_EQ(result, 0);
}

TEST(PercentDifferenceTest, test_a_is_zero)
{
    double result = percent_difference(0, 3);
    EXPECT_EQ(result, 2);
}

TEST(PercentDifferenceTest, test_b_is_zero)
{
    double result = percent_difference(3, 0);
    EXPECT_EQ(result, 2);
}

TEST(PercentDifferenceTest, test_a_is_negative)
{
    double result = percent_difference(-1, 2);
    EXPECT_EQ(result, 2);
}

TEST(PercentDifferenceTest, test_b_is_negative)
{
    double result = percent_difference(1, -2);
    EXPECT_EQ(result, 2);
}

TEST(PercentDifferenceTest, test_both_numbers_negative)
{
    double result = percent_difference(-3, -2);
    EXPECT_EQ(result, 0.4);
}

TEST(PercentDifferenceTest, test_both_numbers_positive)
{
    double result = percent_difference(3, 2);
    EXPECT_EQ(result, 0.4);
}

TEST(PercentDifferenceTest, test_same_positive_numbers)
{
    double result = percent_difference(3, 3);
    EXPECT_EQ(result, 0);
}

TEST(PercentDifferenceTest, test_same_negative_numbers)
{
    double result = percent_difference(-3, -3);
    EXPECT_EQ(result, 0);
}
