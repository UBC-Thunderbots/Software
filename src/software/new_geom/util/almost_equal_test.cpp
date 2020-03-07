#include "software/new_geom/util/almost_equal.h"

#include <gtest/gtest.h>

TEST(AlmostEqualTest, compare_almost_equal_doubles_max_1)
{
    double a = 1.0;
    double b = 1.0 - 5e-16;  // For comparing doubles near 1.0, 5e-16 is small enough
    EXPECT_TRUE(almostEqual(a, b));
}

TEST(AlmostEqualTest, compare_not_almost_equal_doubles_max_1)
{
    double a = 1.0;
    double b =
        1.0 -
        2e-15;  // For comparing doubles near 1.0, 2e-15 is too big to consider equal
    EXPECT_FALSE(almostEqual(a, b));
}

TEST(AlmostEqualTest, compare_almost_equal_doubles_max_10)
{
    double a = 10.0;
    double b = 10.0 - 5e-15;  // For comparing doubles near 10.0, 5e-16 is small enough
    EXPECT_TRUE(almostEqual(a, b));
}

TEST(AlmostEqualTest, compare_not_almost_equal_doubles_max_10)
{
    double a = 10.0;
    double b =
        10.0 -
        2e-14;  // For comparing doubles near 10.0, 2e-14 is too big to consider equal
    EXPECT_FALSE(almostEqual(a, b));
}
