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
