#include "software/new_geom/util/closest_point.h"

#include <gtest/gtest.h>

TEST(ClosestPointTest, three_points)
{
    Point p = closestPointOnLine(Point(0, 0), Point(2, 0), Point(0, 2));
    EXPECT_EQ(p, Point(1, 1));
}

TEST(ClosestPointTest, point_line1)
{
    Point p = closestPointOnLine(Point(0, 0), Line(Point(2, 0), Point(0, 2)));
    EXPECT_EQ(p, Point(1, 1));
}

TEST(ClosestPointTest, point_line2)
{
    Point p = closestPointOnLine(Point(2, 2), Line(Point(3, 0), Point(0, 1.5)));
    EXPECT_EQ(p, Point(1.4, 0.8));
}

TEST(ClosestPointTest, point_on_line)
{
    Point p = closestPointOnLine(Point(4, 4), Line(Point(0, 0), Point(1, 1)));
    EXPECT_EQ(p, Point(4, 4));
}
