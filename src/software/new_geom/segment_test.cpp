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

    EXPECT_TRUE(segment.contains(segment.getStart()));
    EXPECT_TRUE(segment.contains(segment.getEnd()));
}

TEST(SegmentContainsPointTest, vertical_segment_contains_point)
{
    // A point that is very close to being collinear with a vertical segment
    Segment segment(Point(202, 15), Point(202, -15));
    Point point(202.00000000000003, -0.5);

    EXPECT_TRUE(segment.contains(point));
}

TEST(SegmentClosestPointTest, test_closest_lineseg_point_1)
{
    Segment seg(Point(-1, 1), Point(1, 1));

    EXPECT_TRUE((seg.closestPointOnSeg(Point(0, 2)) - Point(0, 1)).length() < 0.00001);
    EXPECT_TRUE((seg.closestPointOnSeg(Point(-2, 1.5)) - Point(-1, 1)).length() <
                0.00001);
}

TEST(SegmentClosestPointTest, test_closest_lineseg_point_2)
{
    Segment seg(Point(-2, 1), Point(1, 2));

    EXPECT_TRUE((seg.closestPointOnSeg(Point(1, 0)) - Point(0.4, 1.8)).length() <
                0.00001);
    EXPECT_TRUE((seg.closestPointOnSeg(Point(-1.4, 1.2)) - Point(-1.4, 1.2)).length() <
                0.00001);
}

TEST(SegmentMidPointTest, test_mid_point_1)
{
    Segment segment(Point(202, 15), Point(202, -15));
    EXPECT_EQ(Point(202, 0), segment.midPoint());
}

TEST(SegmentMidPointTest, test_mid_point_2)
{
    Segment segment(Point(-2, 4), Point(5, 3));
    EXPECT_EQ(Point(1.5, 3.5), segment.midPoint());
}
