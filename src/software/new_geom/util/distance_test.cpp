#include "software/new_geom/util/distance.h"

#include <gtest/gtest.h>

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"

TEST(DistanceTest, point_on_line)
{
    Line l(Point(2, 0), Point(0, 2));
    Point p(1, 1);
    double expected = 0;
    EXPECT_DOUBLE_EQ(distance(l, p), expected);
    EXPECT_DOUBLE_EQ(distance(p, l), expected);
}

TEST(DistanceTest, point_near_vertical_line)
{
    Line l(Point(0, 1), Point(0, 0));
    Point p(2, 0);
    double expected = 2;
    EXPECT_DOUBLE_EQ(distance(l, p), expected);
    EXPECT_DOUBLE_EQ(distance(p, l), expected);
}

TEST(DistanceTest, point_near_line)
{
    Line l(Point(2, 0), Point(0, 2));
    Point p(0, 0);
    double expected = sqrt(2);
    EXPECT_DOUBLE_EQ(distance(l, p), expected);
    EXPECT_DOUBLE_EQ(distance(p, l), expected);
}

TEST(DistanceTest, point_far_from_line)
{
    Line l(Point(0, -4), Point(10, 2));
    Point p(179500, -299164);
    double expected = 348879.57175;
    EXPECT_NEAR(expected, distance(l, p), 1e-5);
    EXPECT_NEAR(expected, distance(p, l), 1e-5);
}
