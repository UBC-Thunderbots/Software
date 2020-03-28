#include "software/new_geom/segment.h"

#include <gtest/gtest.h>

TEST(SegmentContainsPointTest, test_segment_contains_point_no_x_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(0, 1));
    Point point     = Point(0, 0.5);

    EXPECT_TRUE(segment.contains(point));
}

TEST(SegmentContainsPointTest, test_segment_contains_point_no_y_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(1, 0));
    Point point     = Point(0.5, 0);

    EXPECT_TRUE(segment.contains(point));
}

TEST(SegmentContainsPointTest, test_diagonal_segment_contains_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(1, 1);

    EXPECT_TRUE(segment.contains(point));
}

TEST(SegmentContainsPointTest, test_segment_doesnt_contain_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(-2, -2);

    EXPECT_FALSE(segment.contains(point));
}

TEST(SegmentContainsPointTest, test_segment_contains_endpoints)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));

    EXPECT_TRUE(segment.contains(segment.getSegStart()));
    EXPECT_TRUE(segment.contains(segment.getEnd()));
}

TEST(SegmentContainsPointTest, vertical_segment_contains_point)
{
    // A point that is very close to being collinear with a vertical segment
    Segment segment(Point(202, 15), Point(202, -15));
    Point point(202.00000000000003, -0.5);

    EXPECT_TRUE(segment.contains(point));
}
