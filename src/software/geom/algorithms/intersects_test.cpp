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

TEST(IntersectsTest, ray_intersects_stadium_start_straight)
{
    Ray ray(Point(0, 0), Angle::fromDegrees(90));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_not_intersects_stadium_start_straight)
{
    Ray ray(Point(0, 0), Angle::fromDegrees(-90));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_FALSE(intersects(ray, stadium));
    EXPECT_FALSE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_intersects_stadium_end_straight)
{
    Ray ray(Point(9, 7), Angle::fromDegrees(-90));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_not_intersects_stadium_end_straight)
{
    Ray ray(Point(9, 7), Angle::fromDegrees(90));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_FALSE(intersects(ray, stadium));
    EXPECT_FALSE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_intersects_stadium_start_angle)
{
    Ray ray(Point(-0.5, 0), Angle::fromDegrees(75));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_not_intersects_stadium_start_angle)
{
    Ray ray(Point(-0.5, 0), Angle::fromDegrees(95));
    Stadium stadium(Point(2, 3), Point(8, 3), 2);

    EXPECT_FALSE(intersects(ray, stadium));
    EXPECT_FALSE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_intersects_stadium_top)
{
    Ray ray(Point(3, 0.2), Angle::fromDegrees(75));
    Stadium stadium(Point(0, 0), Point(8, 0), 1);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_intersects_stadium_bottom)
{
    Ray ray(Point(3, -0.9), Angle::fromDegrees(-100));
    Stadium stadium(Point(0, 0), Point(8, 0), 1);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_intersects_stadium_bottom_outside)
{
    Ray ray(Point(3, -1.2), Angle::fromDegrees(82));
    Stadium stadium(Point(0, 0), Point(8, 0), 1);

    EXPECT_TRUE(intersects(ray, stadium));
    EXPECT_TRUE(intersects(stadium, ray));
}

TEST(IntersectsTest, ray_not_intersects_stadium_bottom_outside)
{
    Ray ray(Point(3, -1.1), Angle::fromDegrees(-90));
    Stadium stadium(Point(0, 0), Point(8, 0), 1);

    EXPECT_FALSE(intersects(ray, stadium));
    EXPECT_FALSE(intersects(stadium, ray));
}

TEST(IntersectsTest, circle_intersects_stadium_inside_end)
{
    Circle circle(Point(0.1, -3), 3);
    Stadium stadium(Point(8, -3), Point(4, -3), 1);

    EXPECT_TRUE(intersects(circle, stadium));
    EXPECT_TRUE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_not_intersects_stadium_inside_end)
{
    Circle circle(Point(0, -3), 2.9);
    Stadium stadium(Point(8, -3), Point(4, -3), 1);

    EXPECT_FALSE(intersects(circle, stadium));
    EXPECT_FALSE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_intersects_stadium_inside_start)
{
    Circle circle(Point(8, 0.9), 3);
    Stadium stadium(Point(8, -3), Point(4, -3), 1);

    EXPECT_TRUE(intersects(circle, stadium));
    EXPECT_TRUE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_not_intersects_stadium_inside_start)
{
    Circle circle(Point(8, 1.1), 3);
    Stadium stadium(Point(8, -3), Point(4, -3), 1);

    EXPECT_FALSE(intersects(circle, stadium));
    EXPECT_FALSE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_in_stadium)
{
    Circle circle(Point(0, 0), 0.5);
    Stadium stadium(Point(-3, -4), Point(3, 4), 2);

    EXPECT_TRUE(intersects(circle, stadium));
    EXPECT_TRUE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_intersects_stadium)
{
    Circle circle(Point(0, -0.1), 1);
    Stadium stadium(Point(-3, -4), Point(3, -4), 3);

    EXPECT_TRUE(intersects(circle, stadium));
    EXPECT_TRUE(intersects(stadium, circle));
}

TEST(IntersectsTest, circle_not_intersects_stadium)
{
    Circle circle(Point(0, 0.1), 1);
    Stadium stadium(Point(-3, -4), Point(3, -4), 3);

    EXPECT_FALSE(intersects(circle, stadium));
    EXPECT_FALSE(intersects(stadium, circle));
}

TEST(IntersectsTest, segment_intersects_stadium_start)
{
    Segment segment(Point(-0.9, -4), Point(-0.9, 4));
    Stadium stadium(Point(0, 0), Point(5, 0), 1);

    EXPECT_TRUE(intersects(segment, stadium));
    EXPECT_TRUE(intersects(stadium, segment));
}

TEST(IntersectsTest, segment_not_intersects_stadium_start)
{
    Segment segment(Point(-2.1, -4), Point(-2.1, 4));
    Stadium stadium(Point(0, 0), Point(5, 0), 2);

    EXPECT_FALSE(intersects(segment, stadium));
    EXPECT_FALSE(intersects(stadium, segment));
}

TEST(IntersectsTest, segment_intersects_stadium_end)
{
    Segment segment(Point(5.9, -4), Point(5.9, 4));
    Stadium stadium(Point(0, 0), Point(5, 0), 1);

    EXPECT_TRUE(intersects(segment, stadium));
    EXPECT_TRUE(intersects(stadium, segment));
}

TEST(IntersectsTest, segment_inside_stadium)
{
    Segment segment(Point(0, 0.5), Point(0, -0.5));
    Stadium stadium(Point(-2, 0), Point(2, 0), 1);

    EXPECT_TRUE(intersects(segment, stadium));
    EXPECT_TRUE(intersects(stadium, segment));
}

TEST(IntersectsTest, segment_half_inside_stadium)
{
    Segment segment(Point(0.2, 3), Point(0, -0.5));
    Stadium stadium(Point(-2, 0), Point(2, 0), 2);

    EXPECT_TRUE(intersects(segment, stadium));
    EXPECT_TRUE(intersects(stadium, segment));
}

TEST(IntersectsTest, segment_not_inside_stadium)
{
    Segment segment(Point(0.2, 3), Point(0, 6));
    Stadium stadium(Point(-2, 0), Point(2, 0), 2);

    EXPECT_FALSE(intersects(segment, stadium));
    EXPECT_FALSE(intersects(stadium, segment));
}

TEST(IntersectsTest, polygon_first_intersects_stadium_start)
{
    Stadium stadium(Point(2, 0), Point(5, 0), 2);
    Polygon polygon{Point(0.1, 5), Point(0.1, -5), Point(-5, 2)};

    EXPECT_TRUE(intersects(stadium, polygon));
    EXPECT_TRUE(intersects(polygon, stadium));
}

TEST(IntersectsTest, polygon_first_not_intersects_stadium_start)
{
    Stadium stadium(Point(2, 0), Point(5, 0), 3);
    Polygon polygon{Point(-1.1, 5), Point(-1.1, -5), Point(-5, 2)};

    EXPECT_FALSE(intersects(stadium, polygon));
    EXPECT_FALSE(intersects(polygon, stadium));
}

TEST(IntersectsTest, polygon_second_intersects_stadium_start)
{
    Stadium stadium(Point(2, 0), Point(5, 0), 2);
    Polygon polygon{Point(-5, 2), Point(0.1, 5), Point(0.1, -5)};

    EXPECT_TRUE(intersects(stadium, polygon));
    EXPECT_TRUE(intersects(polygon, stadium));
}

TEST(IntersectsTest, polygon_second_not_intersects_stadium_start)
{
    Stadium stadium(Point(2, 0), Point(5, 0), 3);
    Polygon polygon{Point(-5, 2), Point(-1.1, 5), Point(-1.1, -5)};

    EXPECT_FALSE(intersects(stadium, polygon));
    EXPECT_FALSE(intersects(polygon, stadium));
}

TEST(IntersectsTest, stadium_start_intersects_stadium_start)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 2);
    Stadium stadium2(Point(-3.9, 0), Point(-9, 0), 2);

    EXPECT_TRUE(intersects(stadium1, stadium2));
    EXPECT_TRUE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadium_start_not_intersects_stadium_start)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 2);
    Stadium stadium2(Point(-4.1, 0), Point(-9, 0), 2);

    EXPECT_FALSE(intersects(stadium1, stadium2));
    EXPECT_FALSE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadium_start_intersects_stadium_end)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 3);
    Stadium stadium2(Point(0, 10.9), Point(20, 11.9), 2);

    EXPECT_TRUE(intersects(stadium1, stadium2));
    EXPECT_TRUE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadium_start_not_intersects_stadium_end)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 2);
    Stadium stadium2(Point(0, 11.1), Point(20, 11.1), 3);

    EXPECT_FALSE(intersects(stadium1, stadium2));
    EXPECT_FALSE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadium_start_intersects_stadium_span)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 3);
    Stadium stadium2(Point(-10, 10.9), Point(10, 10.9), 2);

    EXPECT_TRUE(intersects(stadium1, stadium2));
    EXPECT_TRUE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadium_start_not_intersects_stadium_span)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 2);
    Stadium stadium2(Point(-7, 11.1), Point(5, 11.1), 3);

    EXPECT_FALSE(intersects(stadium1, stadium2));
    EXPECT_FALSE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadiums_cross)
{
    Stadium stadium1(Point(0, -6), Point(0, 6), 1);
    Stadium stadium2(Point(-6, 0), Point(6, 0), 1);

    EXPECT_TRUE(intersects(stadium1, stadium2));
    EXPECT_TRUE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, stadiums_far)
{
    Stadium stadium1(Point(0, 0), Point(0, 6), 2);
    Stadium stadium2(Point(20, 30), Point(60, 80), 3);

    EXPECT_FALSE(intersects(stadium1, stadium2));
    EXPECT_FALSE(intersects(stadium2, stadium1));
}

TEST(IntersectsTest, segment_cross_stadium)
{
    Segment segment(Point(-2, 0), Point(2, 0));
    Stadium stadium(Point(0, -2), Point(0, 2), 1);

    EXPECT_TRUE(intersects(segment, stadium));
    EXPECT_TRUE(intersects(stadium, segment));
}
