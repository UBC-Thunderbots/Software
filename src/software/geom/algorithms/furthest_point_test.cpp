#include "software/geom/algorithms/furthest_point.h"

#include <gtest/gtest.h>


TEST(FurthestCornerTest, test_rect_furthest_corner_from_point_top_right)
{
    Rectangle r(Point(0, 0), Point(8, 4));
    Point p(0.5, 0.5);
    EXPECT_EQ(furthestPoint(r, p), Point(8, 4));
}

TEST(FurthestCornerTest, test_rect_furthest_corner_from_point_top_left)
{
    Rectangle r(Point(0, 0), Point(8, 4));
    Point p(7, 0.5);
    EXPECT_EQ(furthestPoint(r, p), Point(0, 4));
}

TEST(FurthestCornerTest, test_rect_furthest_corner_from_point_bottom_right)
{
    Rectangle r(Point(0, 0), Point(8, 4));
    Point p(0.5, 3);
    EXPECT_EQ(furthestPoint(r, p), Point(8, 0));
}

TEST(FurthestCornerTest, test_rect_furthest_corner_from_point_bottom_left)
{
    Rectangle r(Point(0, 0), Point(8, 4));
    Point p(7, 3);
    EXPECT_EQ(furthestPoint(r, p), Point(0, 0));
}
