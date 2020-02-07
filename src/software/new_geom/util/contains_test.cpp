#include "software/new_geom/util/contains.h"

#include <gtest/gtest.h>

TEST(SegmentContainsPointTest, test_segment_contains_point_no_x_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(0, 1));
    Point point     = Point(0, 0.5);

    EXPECT_TRUE(contains(segment, point));
}

TEST(SegmentContainsPointTest, test_segment_contains_point_no_y_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(1, 0));
    Point point     = Point(0.5, 0);

    EXPECT_TRUE(contains(segment, point));
}

TEST(SegmentContainsPointTest, test_diagonal_segment_contains_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(1, 1);

    EXPECT_TRUE(contains(segment, point));
}

TEST(SegmentContainsPointTest, test_segment_doesnt_contain_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(-2, -2);

    EXPECT_FALSE(contains(segment, point));
}

TEST(SegmentContainsPointTest, test_segment_contains_endpoints)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));

    EXPECT_TRUE(contains(segment, segment.getSegStart()));
    EXPECT_TRUE(contains(segment, segment.getEnd()));
}

TEST(RayContainsPointTest, test_ray_contains_point_no_x_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, 0.5);

    EXPECT_TRUE(contains(ray, point));
}

TEST(RayContainsPointTest, test_ray_doesnt_contain_point_behind_start)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, -0.5);

    EXPECT_FALSE(contains(ray, point));
}

TEST(RayContainsPointTest, test_ray_doesnt_contain_point)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(-5, 10);

    EXPECT_FALSE(contains(ray, point));
}

TEST(RayContainsPointTest, test_ray_contains_point_no_y_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(1, 0));
    Point point = Point(0.5, 0);

    EXPECT_TRUE(contains(ray, point));
}

TEST(RayContainsPointTest, test_diagonal_ray_contains_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(1, 1);

    EXPECT_TRUE(contains(ray, point));
}

TEST(RayContainsPointTest, test_ray_contains_distant_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(-20, -20);

    EXPECT_TRUE(contains(ray, point));
}

TEST(RayContainsPointTest, test_ray_contains_ray_start)
{
    Ray ray = Ray(Point(2, 2), Vector(-1, -1));

    EXPECT_TRUE(contains(ray, ray.getStart()));
}
