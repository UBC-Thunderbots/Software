#include <gtest/gtest.h>
#include "software/new_geom/util/intersections.h"

TEST(IntersectionsTest, test_segments_single_point_intersection)
{
    Segment a = Segment(Point(2, 2), Point(-2, -2));
    Segment b = Segment(Point(-2, 2), Point(2, -2));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(intersections[0], Point());
}

TEST(IntersectionsTest, test_segments_no_intersection)
{
    Segment a = Segment(Point(5, 2), Point(-2, -2));
    Segment b = Segment(Point(-3, -3), Point(-9, 10));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 0);
}

TEST(IntersectionsTest, test_segments_overlapping)
{
    Segment a = Segment(Point(0, 3), Point(5, 3));
    Segment b = Segment(Point(1, 3), Point(4, 3));
    std::vector<Point> intersections = intersection(a, b);
    EXPECT_EQ(intersections.size(), 2);
    EXPECT_EQ(intersections[0], Point(1, 3));
    EXPECT_EQ(intersections[1], Point(4, 3));
}
