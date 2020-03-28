#include "software/new_geom/util/collinear.h"

#include <gtest/gtest.h>

TEST(CollinearPointsTest, test_collinear_points)
{
    Point p(0, 0);
    Point q(1, 1);
    Point r(5, 5);

    EXPECT_TRUE(collinear(p, q, r));
}

TEST(CollinearPointsTest, test_not_collinear_points)
{
    Point p(0, 0);
    Point q(-2, 5);
    Point r(3, -9);

    EXPECT_FALSE(collinear(p, q, r));
}

TEST(CollinearPointsTest, test_points_collinear_two_identical)
{
    EXPECT_TRUE(collinear(Point(), Point(5, 1), Point(5, 1)));
}

TEST(CollinearPointsTest, test_points_collinear_all_identical)
{
    EXPECT_TRUE(collinear(Point(-4, 3), Point(-4, 3), Point(-4, 3)));
}

TEST(CollinearPointsTest, small_double_precision_error_collinear)
{
    // Make sure small double precision error does not affect collinear
    Point p(7.0 + 1.0 / 3.0, 2);
    Point q(5, -5);
    Point r(8, 4);
    EXPECT_TRUE(collinear(p, q, r));
}

TEST(CollinearPointsTest, vertically_collinear_points)
{
    Point p(202, 15);
    Point q(202, -15);
    Point r(202.00000000000003, -0.5);
    EXPECT_TRUE(collinear(p, q, r));
}
