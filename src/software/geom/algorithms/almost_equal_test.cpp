#include "software/geom/algorithms/almost_equal.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "software/geom/geom_constants.h"

// Increments number towards direction by distance times
double ulpsIncrement(double number, double direction, int distance)
{
    double result = number;
    for (int i = 0; i < distance; i++)
    {
        result = std::nextafter(result, direction);
    }
    return result;
}

TEST(AlmostEqualTest, ten_ulps_greater_than_zero)
{
    double a = 0.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::max(), 10);
    // Although a and b are 10 ulps apart, their difference is less than the fixed epsilon
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 5));
}

TEST(AlmostEqualTest, ten_ulps_less_than_zero)
{
    double a = 0.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::lowest(), 10);
    // Although a and b are 10 ulps apart, their difference is less than the fixed epsilon
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 5));
}

TEST(AlmostEqualTest, twenty_ulps_greater_than_one)
{
    double a = 1.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::max(), 20);
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 20));
    EXPECT_FALSE(almostEqual(a, b, FIXED_EPSILON, 19));
}

TEST(AlmostEqualTest, twenty_ulps_less_than_one)
{
    double a = 1.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::lowest(), 20);
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 20));
    EXPECT_FALSE(almostEqual(a, b, FIXED_EPSILON, 19));
}

TEST(AlmostEqualTest, twenty_ulps_greater_than_ten)
{
    double a = 10.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::max(), 20);
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 20));
    EXPECT_FALSE(almostEqual(a, b, FIXED_EPSILON, 19));
}

TEST(AlmostEqualTest, twenty_ulps_less_than_ten)
{
    double a = 10.0;
    double b = ulpsIncrement(a, std::numeric_limits<double>::lowest(), 20);
    EXPECT_TRUE(almostEqual(a, b, FIXED_EPSILON, 20));
    EXPECT_FALSE(almostEqual(a, b, FIXED_EPSILON, 19));
}

TEST(AlmostEqualTest, compare_across_zero)
{
    double a = ulpsIncrement(0.0, std::numeric_limits<double>::max(), 10);
    double b = ulpsIncrement(0.0, std::numeric_limits<double>::lowest(), 10);
    // Return false for comparing values across zero when difference is greater than fixed
    // epsilon
    EXPECT_FALSE(almostEqual(a, b, 0.0, 10));
}
