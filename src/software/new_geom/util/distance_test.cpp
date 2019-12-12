#include "software/new_geom/util/distance.h"

#include <gtest/gtest.h>

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"

TEST(DistanceTest, vertical_line_point)
{
    Line l(Point(0, 1), Point(0, 0));
    Point p(2, 0);
    EXPECT_DOUBLE_EQ(distance(l, p), 2);
    EXPECT_DOUBLE_EQ(distance(p, l), 2);
}

TEST(DistanceTest, line_point1)
{
    Line l(Point(2, 0), Point(0, 2));
    Point p(0, 0);
    EXPECT_DOUBLE_EQ(distance(l, p), sqrt(2));
}

TEST(DistanceTest, line_point2)
{
    Line l(Point(1, -1), Point(5, -2));
    Point p(2, -3);
    EXPECT_NEAR(1.69775, distance(l, p), 1e-5);

    p = Point(2, 1);
    EXPECT_NEAR(2.18282, distance(l, p), 1e-5);

    p = Point(2, 0);
    EXPECT_NEAR(1.21268, distance(l, p), 1e-5);
}
