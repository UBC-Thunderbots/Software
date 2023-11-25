#include "software/geom/algorithms/distance.h"

#include <gtest/gtest.h>

#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

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
    Point p(2.1, 0);
    double expected = 2.1;
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
    double expected =
        std::abs(-6.0 / 10.0 * p.x() + p.y() + 4) / std::hypot(-6.0 / 10.0, 1);
    EXPECT_NEAR(distance(l, p), expected, 1e-10);
    EXPECT_NEAR(distance(p, l), expected, 1e-10);
}

TEST(DistanceTest, same_points)
{
    Point p1(3, 5);
    Point p2(3, 5);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(p1, p2), expected);
}

TEST(DistanceTest, different_points)
{
    Point p1(-1, -7);
    Point p2(4, 5);
    double expected = 13.0;
    EXPECT_DOUBLE_EQ(distance(p1, p2), expected);
}

TEST(DistanceTest, point_on_segment_end)
{
    Point p(12, 5);
    Segment s(Point(12, 5), Point(-1, -20));
    double expected = 0;
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_on_segment_middle)
{
    Point p(1, 1);
    Segment s(Point(-3, -3), Point(2, 2));
    double expected = 0;
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_off_segment_in_direction)
{
    Point p(-8, 5);
    Segment s(Point(-4, 4), Point(4, 2));
    double expected = std::sqrt(17);
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_off_segment_closest_to_segment_middle)
{
    Point p(-2, 7);
    Segment s(Point(-4, 4), Point(4, 2));
    double expected = std::abs(1.0 / 4.0 * -2 + 7 + -3) / std::hypot(1.0 / 4.0, 1);
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_off_segment_closest_to_segment_end)
{
    Point p(2, 12);
    Segment s(Point(-7, 2), Point(1, 8));
    double expected = std::hypot(4, 1);
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_off_degenerate_segment)
{
    Point p(5, 3);
    Segment s(Point(-2, 2), Point(-2, 2));
    double expected = std::hypot(7, 1);
    EXPECT_DOUBLE_EQ(distance(p, s), expected);
    EXPECT_DOUBLE_EQ(distance(s, p), expected);
}

TEST(DistanceTest, point_in_polygon)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});
    Point point(0.5, 0.5);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(point, polygon), expected);
    EXPECT_DOUBLE_EQ(distance(polygon, point), expected);
}

TEST(DistanceTest, point_on_polygon_edge)
{
    Polygon polygon({Point(-1, 1), Point(1, 0), Point(1, 1), Point(0.5, 2), Point(0, 1)});
    Point point(0.25, 1.5);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(point, polygon), expected);
    EXPECT_DOUBLE_EQ(distance(polygon, point), expected);
}

TEST(DistanceTest, point_on_polygon_vertex)
{
    Polygon polygon({Point(-1, 1), Point(1, 0), Point(1, 1), Point(0.5, 2), Point(0, 1)});
    Point point(0.5, 2);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(point, polygon), expected);
    EXPECT_DOUBLE_EQ(distance(polygon, point), expected);
}

TEST(DistanceTest, point_off_polygon_closest_to_vertex)
{
    Polygon polygon({Point(-1, 1), Point(1, 0), Point(1, 1), Point(0.5, 2), Point(0, 1)});
    Point point(-2, 2);
    double expected = std::sqrt(2);
    EXPECT_DOUBLE_EQ(distance(point, polygon), expected);
    EXPECT_DOUBLE_EQ(distance(polygon, point), expected);
}

TEST(DistanceTest, point_near_polygon_closest_to_edge)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});
    Point point(0.5, 1.1);
    double expected = 0.1;
    EXPECT_NEAR(distance(point, polygon), expected, FIXED_EPSILON);
    EXPECT_NEAR(distance(polygon, point), expected, FIXED_EPSILON);
}

TEST(DistanceTest, point_far_from_polygon)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});
    Point point(-5, -5);
    double expected = std::hypot(-5, -5);
    EXPECT_DOUBLE_EQ(distance(point, polygon), expected);
    EXPECT_DOUBLE_EQ(distance(polygon, point), expected);
}

TEST(DistanceTest, point_in_rectangle)
{
    Point p(1, 2.1);
    Rectangle r({0, 2}, {2, 4});
    double expected = 0;
    EXPECT_DOUBLE_EQ(distance(p, r), expected);
    EXPECT_DOUBLE_EQ(distance(r, p), expected);
}


TEST(DistanceTest, point_below_rectangle)
{
    Point p(1, 1);
    Rectangle r({0, 2}, {2, 4});
    double expected = 1.0;
    EXPECT_DOUBLE_EQ(distance(p, r), expected);
    EXPECT_DOUBLE_EQ(distance(r, p), expected);
}

TEST(DistanceTest, point_above_rectangle)
{
    Point p(1, 5);
    Rectangle r({0, 2}, {2, 4});
    double expected = 1.0;
    EXPECT_DOUBLE_EQ(distance(p, r), expected);
    EXPECT_DOUBLE_EQ(distance(r, p), expected);
}

TEST(DistanceTest, point_left_rectangle)
{
    Point p(-1, 3);
    Rectangle r({0, 2}, {2, 4});
    double expected = 1.0;
    EXPECT_DOUBLE_EQ(distance(p, r), expected);
    EXPECT_DOUBLE_EQ(distance(r, p), expected);
}

TEST(DistanceTest, point_right_rectangle)
{
    Point p(3, 3);
    Rectangle r({0, 2}, {2, 4});
    double expected = 1.0;
    EXPECT_DOUBLE_EQ(distance(p, r), expected);
    EXPECT_DOUBLE_EQ(distance(r, p), expected);
}

TEST(DistanceTest, point_in_circle)
{
    Circle circle(Point(2, 3), 4);
    Point point(3, 4);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(point, circle), expected);
    EXPECT_DOUBLE_EQ(distance(circle, point), expected);
}

TEST(DistanceTest, point_on_circle_circumference)
{
    Circle circle(Point(2.5, 3), 4);
    Point point(6.5, 3);
    double expected = 0.0;
    EXPECT_DOUBLE_EQ(distance(point, circle), expected);
    EXPECT_DOUBLE_EQ(distance(circle, point), expected);
}

TEST(DistanceTest, point_outside_circle)
{
    Circle circle(Point(2.5, -2), 1);
    Point point(6.5, -2);
    double expected = 3.0;
    EXPECT_DOUBLE_EQ(distance(point, circle), expected);
    EXPECT_DOUBLE_EQ(distance(circle, point), expected);
}

TEST(DistanceTest, point_on_segment_end_squared)
{
    Point p(12, 5);
    Segment s(Point(12, 5), Point(-1, -20));
    double expected = 0;
    EXPECT_DOUBLE_EQ(distanceSquared(p, s), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(s, p), expected);
}

TEST(DistanceTest, point_on_segment_middle_squared)
{
    Point p(1, 1);
    Segment s(Point(-3, -3), Point(2, 2));
    double expected = 0;
    EXPECT_DOUBLE_EQ(distanceSquared(p, s), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(s, p), expected);
}

TEST(DistanceTest, point_off_segment_in_direction_squared)
{
    Point p(-8, 5);
    Segment s(Point(-4, 4), Point(4, 2));
    double expected = 17;
    EXPECT_DOUBLE_EQ(distanceSquared(p, s), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(s, p), expected);
}

TEST(DistanceTest, point_off_segment_closest_to_segment_middle_squared)
{
    Point p(-2, 7);
    Segment s(Point(-4, 4), Point(4, 2));
    double expected =
        std::pow(std::abs(1.0 / 4.0 * -2 + 7 + -3), 2) / (std::pow(1.0 / 4.0, 2) + 1);
    EXPECT_DOUBLE_EQ(distanceSquared(p, s), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(s, p), expected);
}

TEST(DistanceTest, point_off_segment_closest_to_segment_end_squared)
{
    Point p(2, 12);
    Segment s(Point(-7, 2), Point(1, 8));
    double expected = 17;
    EXPECT_DOUBLE_EQ(distanceSquared(p, s), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(s, p), expected);
}

TEST(DistanceTest, same_points_squared)
{
    Point p1(8, 4);
    Point p2(8, 4);
    double expected = 0;
    EXPECT_DOUBLE_EQ(distanceSquared(p1, p2), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(p2, p1), expected);
}

TEST(DistanceTest, different_points_squared)
{
    Point p1(8, 4);
    Point p2(56, 25);
    double expected = 2745;
    EXPECT_DOUBLE_EQ(distanceSquared(p1, p2), expected);
    EXPECT_DOUBLE_EQ(distanceSquared(p2, p1), expected);
}
