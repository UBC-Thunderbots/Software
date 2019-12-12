#include "software/new_geom/util/closest_point.h"

#include <gtest/gtest.h>

TEST(ClosestPointTest, point_line_negative_slope)
{
    Point p(0, 0);
    Line l(Point(2, 0), Point(0, 2));
    Point expected(1, 1);
    EXPECT_EQ(closestPointOnLine(p, l), expected);
    EXPECT_EQ(closestPointOnLine(l, p), expected);
}

TEST(ClosestPointTest, point_line_positive_slope)
{
    Point p(-2, 2.5);
    Line l(Point(1, 0), Point(2, 2));
    Point expected(1.4, 0.8);
    EXPECT_EQ(closestPointOnLine(p, l), expected);
    EXPECT_EQ(closestPointOnLine(l, p), expected);
}

TEST(ClosestPointTest, point_on_line)
{
    Point p(4, 4);
    Line l(Point(0, 0), Point(1, 1));
    Point expected(4, 4);
    EXPECT_EQ(closestPointOnLine(p, l), expected);
    EXPECT_EQ(closestPointOnLine(l, p), expected);
}
