#include "software/geom/segment.h"

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

TEST(SegmentIntersectsTest, segments_overlapping)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-9, 4}, {-8.5, 3});
    EXPECT_TRUE(s1.intersects(s2));
    EXPECT_TRUE(s2.intersects(s1));
}

TEST(SegmentIntersectsTest, segments_end_points_overlapping)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-10, 6}, {10, 3});
    EXPECT_TRUE(s1.intersects(s2));
    EXPECT_TRUE(s2.intersects(s1));
}

TEST(SegmentIntersectsTest, segments_parallel)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-9, 6}, {-7.5, 3});
    EXPECT_FALSE(s1.intersects(s2));
    EXPECT_FALSE(s2.intersects(s1));
}

TEST(SegmentIntersectsTest, segments_intersecting)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-10, 0}, {-6, 8});
    EXPECT_TRUE(s1.intersects(s2));
    EXPECT_TRUE(s2.intersects(s1));
}

TEST(SegmentIntersectsTest, segments_far_from_each_other)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({20000, 100000}, {40000, -20000});
    EXPECT_FALSE(s1.intersects(s2));
    EXPECT_FALSE(s2.intersects(s1));
}

TEST(SegmentIntersectsTest, close_parallel_segments_not_intersecting)
{
    // This is a test from a bug found
    Segment s1({1.049, -1.049}, {1.95, -1.049});
    Segment s2({2, -1}, {1, -1});
    EXPECT_FALSE(s1.intersects(s2));
    EXPECT_FALSE(s2.intersects(s1));
}

TEST(SegmentIntersectionsTest, test_segments_single_point_intersection)
{
    Segment a                        = Segment(Point(2, 2), Point(-2, -2));
    Segment b                        = Segment(Point(-2, 2), Point(2, -2));
    std::vector<Point> intersections = a.intersection(b);
    EXPECT_EQ(intersections.size(), 1);
    EXPECT_EQ(intersections[0], Point());
}

TEST(SegmentIntersectionsTest, test_segments_no_intersection)
{
    Segment a                        = Segment(Point(5, 2), Point(-2, -2));
    Segment b                        = Segment(Point(-3, -3), Point(-9, 10));
    std::vector<Point> intersections = a.intersection(b);
    EXPECT_EQ(intersections.size(), 0);
}

TEST(SegmentIntersectionsTest, test_segments_overlapping)
{
    Segment a                        = Segment(Point(0, 3), Point(5, 3));
    Segment b                        = Segment(Point(1, 3), Point(4, 3));
    std::vector<Point> intersections = a.intersection(b);
    EXPECT_EQ(intersections.size(), 2);
    EXPECT_EQ(intersections[0], Point(1, 3));
    EXPECT_EQ(intersections[1], Point(4, 3));
}
