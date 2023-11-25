#include "software/geom/algorithms/intersects.h"

#include <gtest/gtest.h>

#include "software/geom/rectangle.h"
#include "software/geom/triangle.h"

TEST(IntersectsTest, polygon_segment_intersecting_on_edge)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({-7, 0}, {-4, 1});
    EXPECT_TRUE(intersects(p, s));
    EXPECT_TRUE(intersects(s, p));
}

TEST(IntersectsTest, polygon_segment_intersecting_middle)
{
    Polygon p({{-1, -3}, {-1, 4}, {1, 4}, {1, -3}});
    Segment s({-7, 0}, {7, 0});
    EXPECT_TRUE(intersects(p, s));
    EXPECT_TRUE(intersects(s, p));
}

TEST(IntersectsTest, polygon_segment_intersecting_on_vertex)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({-2, 0}, {0, 2});
    EXPECT_TRUE(intersects(p, s));
    EXPECT_TRUE(intersects(s, p));
}

TEST(IntersectsTest, polygon_far_from_segment_not_intersecting)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({100000, 20000}, {-5000, 200});
    EXPECT_FALSE(intersects(p, s));
    EXPECT_FALSE(intersects(s, p));
}

TEST(IntersectsTest, polygon_near_segment_not_intersecting)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({-6, 2.5}, {-1, 1.5});
    EXPECT_FALSE(intersects(p, s));
    EXPECT_FALSE(intersects(s, p));
}

TEST(IntersectsTest, segment_overlapping_polygon_edge)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({2, -2.5}, {8, -9.5});
    EXPECT_TRUE(intersects(p, s));
    EXPECT_TRUE(intersects(s, p));
}

TEST(IntersectsTest, segment_contained_in_polygon)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Segment s({0, -2}, {4, 1});
    EXPECT_TRUE(intersects(p, s));
    EXPECT_TRUE(intersects(s, p));
}

TEST(IntersectsTest, ray_inside_polygon)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Ray r({2, -2}, Angle::fromDegrees(20));
    EXPECT_TRUE(intersects(p, r));
    EXPECT_TRUE(intersects(r, p));
}

TEST(IntersectsTest, ray_outside_polygon_intersecting)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Ray r({-2, -6}, Angle::fromDegrees(100));
    EXPECT_TRUE(intersects(p, r));
    EXPECT_TRUE(intersects(r, p));
}

TEST(IntersectsTest, ray_outside_polygon_not_intersecting)
{
    Polygon p({{-6, 2}, {-1, 1}, {10, 7}, {5, -6}, {-5, -3}});
    Ray r({-2, -6}, Angle::fromDegrees(180));
    EXPECT_FALSE(intersects(p, r));
    EXPECT_FALSE(intersects(r, p));
}

TEST(IntersectsTest, circle_tangent_to_triangle_edge)
{
    Triangle t({-5, 0}, {5, 0}, {2, 5});
    Circle c({0, -1}, 1);
    EXPECT_FALSE(intersects(t, c));
    EXPECT_FALSE(intersects(c, t));
}

TEST(IntersectsTest, circle_completely_inside_triangle)
{
    Triangle t({-10, 0}, {10, 0}, {0, 15});
    Circle c({0, 5}, 1);
    EXPECT_TRUE(intersects(t, c));
    EXPECT_TRUE(intersects(c, t));
}

TEST(IntersectsTest, circle_tangent_to_triangle_vertice)
{
    Triangle t({-5, -5}, {5, -5}, {0, 0});
    Circle c({0, 1}, 1);
    EXPECT_FALSE(intersects(t, c));
    EXPECT_FALSE(intersects(c, t));
}

TEST(IntersectsTest, circle_near_triangle_not_intersecting)
{
    Triangle t({-8, -5}, {0, 0}, {-3, -2});
    Circle c({5, 5}, 2);
    EXPECT_FALSE(intersects(t, c));
    EXPECT_FALSE(intersects(c, t));
}

TEST(IntersectsTest, circle_far_from_triangle_not_intersecting)
{
    Triangle t({-8, 5}, {0, 0}, {-3, -2});
    Circle c({100000, -250000}, 1);
    EXPECT_FALSE(intersects(t, c));
    EXPECT_FALSE(intersects(c, t));
}

TEST(IntersectsTest, circle_far_from_triangle_intersecting)
{
    Triangle t({-8, 5}, {0, 0}, {-3, -2});
    Circle c({100000, -250000}, 300000);
    EXPECT_TRUE(intersects(t, c));
    EXPECT_TRUE(intersects(c, t));
}

TEST(IntersectsTest, circle_origin_inside_triangle_tangent)
{
    Triangle t({-2, -2}, {2, -2}, {0, 1});
    Circle c({0, -1}, 1);
    EXPECT_TRUE(intersects(t, c));
    EXPECT_TRUE(intersects(c, t));
}

TEST(IntersectsTest, circle_origin_inside_triangle_overlapping)
{
    Triangle t({-2, -2}, {2, -2}, {0, 1});
    Circle c({0, -1}, 5);
    EXPECT_TRUE(intersects(t, c));
    EXPECT_TRUE(intersects(c, t));
}

TEST(IntersectsTest, circle_origin_outside_triangle_intersect)
{
    Triangle t({-2, -2}, {2, -2}, {0, 1});
    Circle c({0, -3}, 1.5);
    EXPECT_TRUE(intersects(t, c));
    EXPECT_TRUE(intersects(c, t));
}

TEST(IntersectsTest, circle_inside_rectangle)
{
    Rectangle r({-5, -2}, {3, 4});
    Circle c({0, 0}, 1);
    EXPECT_TRUE(intersects(r, c));
    EXPECT_TRUE(intersects(c, r));
}

TEST(IntersectsTest, circle_outside_rectangle_intersecting)
{
    Rectangle r({-5, -2}, {3, 4});
    Circle c({4, -5}, 3.5);
    EXPECT_TRUE(intersects(r, c));
    EXPECT_TRUE(intersects(c, r));
}

TEST(IntersectsTest, circle_outside_rectangle_not_intersecting)
{
    Rectangle r({-5, -2}, {3, 4});
    Circle c({4, -5}, 2);
    EXPECT_FALSE(intersects(r, c));
    EXPECT_FALSE(intersects(c, r));
}

TEST(IntersectsTest, circles_same_origin)
{
    Circle c1({4, -5}, 3);
    Circle c2({4, -5}, 2);
    EXPECT_TRUE(intersects(c1, c2));
}

TEST(IntersectsTest, circle_origin_in_other_circle)
{
    Circle c1({4, -5}, 3);
    Circle c2({6, -5}, 1);
    EXPECT_TRUE(intersects(c1, c2));
}

TEST(IntersectsTest, circles_with_origins_not_overlapping)
{
    Circle c1({-6, 2}, 4);
    Circle c2({4, -5}, 9);
    EXPECT_TRUE(intersects(c1, c2));
}

TEST(IntersectsTest, segment_completely_inside_circle)
{
    Segment s({-8, 4}, {-4, 0});
    Circle c({-6, 2}, 4);
    EXPECT_TRUE(intersects(s, c));
    EXPECT_TRUE(intersects(c, s));
}

TEST(IntersectsTest, segment_with_points_on_circle_edge)
{
    Segment s({-2, 2}, {-6, 6});
    Circle c({-6, 2}, 4);
    EXPECT_TRUE(intersects(s, c));
    EXPECT_TRUE(intersects(c, s));
}

TEST(IntersectsTest, segment_circle_intersecting)
{
    Segment s({-8, 2}, {-10, 6});
    Circle c({-6, 2}, 4);
    EXPECT_TRUE(intersects(s, c));
    EXPECT_TRUE(intersects(c, s));
}

TEST(IntersectsTest, segment_circle_intersecting_middle)
{
    Segment s({-10, 2}, {10, 2});
    Circle c({-4, 2}, 4);
    EXPECT_TRUE(intersects(s, c));
    EXPECT_TRUE(intersects(c, s));
}

TEST(IntersectsTest, segments_overlapping)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-9, 4}, {-8.5, 3});
    EXPECT_TRUE(intersects(s1, s2));
    EXPECT_TRUE(intersects(s1, s2));
}

TEST(IntersectsTest, segments_end_points_overlapping)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-10, 6}, {10, 3});
    EXPECT_TRUE(intersects(s1, s2));
    EXPECT_TRUE(intersects(s2, s1));
}

TEST(IntersectsTest, segments_parallel)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-9, 6}, {-7.5, 3});
    EXPECT_TRUE(!intersects(s1, s2));
    EXPECT_TRUE(!intersects(s2, s1));
}

TEST(IntersectsTest, segments_intersecting)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({-10, 0}, {-6, 8});
    EXPECT_TRUE(intersects(s1, s2));
    EXPECT_TRUE(intersects(s2, s1));
}

TEST(IntersectsTest, segments_far_from_each_other)
{
    Segment s1({-8, 2}, {-10, 6});
    Segment s2({20000, 100000}, {40000, -20000});
    EXPECT_FALSE(intersects(s1, s2));
    EXPECT_FALSE(intersects(s2, s1));
}

TEST(IntersectsTest, close_parallel_segments_not_intersecting)
{
    // This is a test from a bug found
    Segment s1({1.049, -1.049}, {1.95, -1.049});
    Segment s2({2, -1}, {1, -1});
    EXPECT_FALSE(intersects(s1, s2));
    EXPECT_FALSE(intersects(s2, s1));
}

TEST(IntersectsTest,
     test_ray_intersect_position_and_direction_intersect_not_just_direction)
{
    // Test to ensure that intersects(Ray, Segment) does not use ray.getDirection() as
    // a point along the ray (Should be ray.getStart() + ray.GetDirection())
    Segment segment = Segment(Point(-1, 1), Point(1, 1));

    Ray position_and_direction = Ray(Point(-2, 0), Vector(0, 1));
    Ray just_direction         = Ray(Point(0, 0), Vector(0, 1));

    EXPECT_EQ(intersects(position_and_direction, segment), false);
    EXPECT_EQ(intersects(just_direction, segment), true);
}

TEST(IntersectsTest, ray_intersects_segment_edge)
{
    Ray r({5, 2}, Angle::zero());
    Segment s({5, -5}, {8, 4});
    EXPECT_TRUE(intersects(r, s));
    EXPECT_TRUE(intersects(s, r));
}

TEST(IntersectsTest, ray_starting_from_segment_edge)
{
    Ray r({6, -2}, Angle::fromDegrees(30));
    Segment s({5, -5}, {8, 4});
    EXPECT_TRUE(intersects(r, s));
    EXPECT_TRUE(intersects(s, r));
}

TEST(IntersectsTest, ray_overlapping_segment)
{
    Ray r({6, -2}, Angle::fromRadians(std::atan2(3, 1)));
    Segment s({5, -5}, {8, 4});
    EXPECT_TRUE(intersects(r, s));
    EXPECT_TRUE(intersects(s, r));
}

TEST(IntersectsTest, ray_starting_from_segment_end)
{
    Ray r({8, 4}, Angle::fromDegrees(30));
    Segment s({5, -5}, {8, 4});
    EXPECT_TRUE(intersects(r, s));
    EXPECT_TRUE(intersects(s, r));
}

TEST(IntersectsTest, ray_parallel_to_segment_not_intersecting)
{
    Ray r({-2, -2}, Angle::fromRadians(std::atan2(3, 1)));
    Segment s({5, -5}, {8, 4});
    EXPECT_FALSE(intersects(r, s));
    EXPECT_FALSE(intersects(s, r));
}

TEST(IntersectsTest, ray_far_from_segment)
{
    Ray r({-150000, 500000}, Angle::fromRadians(std::atan2(-4, -2)));
    Segment s({5, -5}, {8, 4});
    EXPECT_FALSE(intersects(r, s));
    EXPECT_FALSE(intersects(s, r));
}
