#include "software/new_geom/util/contains.h"

#include <gtest/gtest.h>

TEST(ContainsTest, segment_in_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {4, 0});
    EXPECT_TRUE(contains(c, s));
}

TEST(ContainsTest, segment_one_point_in_circle_other_point_on_circle_edge)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {5, 1});
    EXPECT_TRUE(contains(c, s));
}

TEST(ContainsTest, segment_one_point_in_circle_other_point_out)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {20, -10});
    EXPECT_FALSE(contains(c, s));
}


TEST(ContainsTest, segment_both_points_on_circle_edge)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({1, 5}, {5, 1});
    EXPECT_TRUE(contains(c, s));
}

TEST(ContainsTest,
     segment_one_point_on_circle_edge_other_point_not_passing_through_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({1, 5}, {6, 6});
    EXPECT_FALSE(contains(c, s));
}

TEST(ContainsTest, segment_both_points_out_of_circle_passing_through)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-4, 4}, {2, -4});
    EXPECT_FALSE(contains(c, s));
}

TEST(ContainsTest, segment_completely_out_of_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-50, -50}, {30, -30});
    EXPECT_FALSE(contains(c, s));
}

TEST(ContainsTest, zero_circle_contains_point)
{
    Circle zero = Circle();
    Point p     = Point(1, 1);
    EXPECT_FALSE(contains(zero, p));
    EXPECT_TRUE(contains(zero, Point()));
}

TEST(ContainsTest, circle_at_origin_contains_point)
{
    Circle c = Circle(Point(), 5);
    EXPECT_FALSE(contains(c, Point(-6, -6)));
    EXPECT_TRUE(contains(c, Point(-2, 3)));
}

TEST(ContainsTest, circle_not_at_origin_contains_point)
{
    Circle d = Circle(Point(-5, -5), 5);
    EXPECT_FALSE(contains(d, Point()));
    EXPECT_TRUE(contains(d, Point(-10, -5)));
}

TEST(ContainsTest, test_polygon_triangle_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{0.9f, 0.9f};
    Polygon triangle{p1, p2, p3};
    EXPECT_TRUE(contains(triangle, point));
}

TEST(ContainsTest, test_polygon_triangle_not_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};
    EXPECT_FALSE(contains(triangle, point));
}

TEST(ContainsTest, test_polygon_hexagon_contains_point)
{
    // Hexagon centered at origin with the following points
    Polygon hexagon{{0.0f, 2.0f},    // top vertex
                    {2.0f, 1.0f},    // top right vertex
                    {2.0f, -1.0f},   // bottom right vertex
                    {0.0f, -2.0f},   // bottom vertex
                    {-2.0f, -1.0f},  // bottom left vertex
                    {-2.0f, 1.0f}};  // top left vertex

    EXPECT_TRUE(contains(hexagon, Point()));
    EXPECT_FALSE(contains(hexagon, Point(0, 2.01)));
    EXPECT_FALSE(contains(hexagon, Point(0, -2.01)));
    EXPECT_FALSE(contains(hexagon, Point(2.01, 0)));
    EXPECT_FALSE(contains(hexagon, Point(-2.01, 0)));
    EXPECT_FALSE(contains(
        hexagon, Point(2.0f, 0.0f)));  // on right edge, see NOTE on Polygon::contains
    EXPECT_TRUE(contains(hexagon, Point(-2, 0)));  // on left edge
    EXPECT_TRUE(
        contains(hexagon, Point(-2, -1)));  // the bottom left vertex of the hexagon
    EXPECT_TRUE(contains(hexagon, Point(1, -1)));
    EXPECT_TRUE(contains(hexagon, Point(-1.5, 0.75)));
}

TEST(ContainsTest, test_self_intersecting_polygon_contains)
{
    /*
     *  2    *-------*
     *       |       |
     *  1    *-------*
     *       |
     *  0    *
     *       0       2
     */
    // Self intersecting polygon, each asterisk on the diagram is a point making up the
    // polygon
    Polygon intersecting_poly{
        {0.0f, 0.0f}, {0.0f, 2.0f}, {2.0f, 2.0f}, {2.0f, 1.0f}, {0.0f, 1.0f}};

    EXPECT_FALSE(contains(intersecting_poly,
                          Point()));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(contains(intersecting_poly, Point(2, 0)));
    EXPECT_FALSE(contains(intersecting_poly, Point(0.5, 0.5)));
    EXPECT_FALSE(
        contains(intersecting_poly,
                 Point(0, 0.5)));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_TRUE(contains(intersecting_poly, Point(0, 1)));
    EXPECT_TRUE(contains(intersecting_poly, Point(1, 1.5)));
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(contains(intersecting_poly, Point(1, 1)));
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(1, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(contains(intersecting_poly, Point(0, 1.5)));
}

TEST(ContainsTest, test_complex_self_intersecting_polygon_contains)
{
    /*
     *  2            *-------*
     *               |       |
     *  1            *-------*
     *               |
     *  0            *
     *               |
     *  -1   *-------*
     *       |       |
     *  -2   *-------*
     *
     *      -2       0       2
     */
    // Self intersecting polygon, each asterisk on the diagram is a point making up the
    // polygon
    Polygon intersecting_poly{{0.0f, 0.0f},   {0.0f, 2.0f},   {2.0f, 2.0f},
                              {2.0f, 1.0f},   {0.0f, 1.0f},   {-0.0f, -2.0f},
                              {-2.0f, -2.0f}, {-2.0f, -1.0f}, {0.0f, -1.0f}};

    EXPECT_FALSE(contains(intersecting_poly,
                          Point()));  // on a "right" edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly, Point(2, 0)));
    EXPECT_FALSE(contains(intersecting_poly, Point(0.5, 0.5)));
    EXPECT_FALSE(
        contains(intersecting_poly,
                 Point(0, 0.5)));  // on a "right" edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, 2)));  // on a top edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(1, 2)));  // on a top edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly, Point(-2, 0)));
    EXPECT_FALSE(contains(intersecting_poly, Point(-0.5, -0.5)));
    EXPECT_FALSE(
        contains(intersecting_poly,
                 Point(0, -0.5)));  // on a "right" edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, -2)));  // on a top edge, see NOTE on poly::contains
    EXPECT_FALSE(contains(intersecting_poly, Point(0, -1)));
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(-1, -1)));  // on a "top" edge, see NOTE on poly::contains
    EXPECT_FALSE(
        contains(intersecting_poly,
                 Point(0, -1.5)));  // on a "right" edge, see NOTE on poly::contains
    EXPECT_TRUE(contains(intersecting_poly, Point(-1, -2)));
    EXPECT_TRUE(contains(intersecting_poly, Point(0, 1)));
    EXPECT_TRUE(contains(intersecting_poly, Point(1, 1.5)));
    EXPECT_TRUE(contains(intersecting_poly, Point(1, 1)));
    EXPECT_TRUE(contains(intersecting_poly, Point(0, 1.5)));
    EXPECT_TRUE(contains(intersecting_poly, Point(-1, -1.5)));
}

TEST(ContainsTest, test_self_intersecting_loop_polygon_contains)
{
    /*
     *  3    *------------------------------------------------*
     *       |                                                |
     *       |                                                |
     *  2    |       *--------------------------------*       |
     *       |             ------           ------            |
     *       |                  ----      ----                |
     *  1    |                       ------                   |
     *       |                ------      ------              |
     *       |         ------                   -------       |
     *  0    *------                                   -------*
     *      -3      -2       -1       0       1       2       3
     */
    // Self intersecting polygon, each asterisk on the diagram is a point making up the
    // polygon
    Polygon intersecting_poly{{-3.0f, 0.0f}, {-3.0f, 3.0f}, {3.0f, 3.0f},
                              {3.0f, 0.0f},  {-2.0f, 2.0f}, {2.0f, 2.0f}};

    EXPECT_FALSE(
        contains(intersecting_poly,
                 Point(3, 2)));  // on a right edge, see NOTE on Polygon::contains
    EXPECT_TRUE(contains(intersecting_poly,
                         Point(2, 2)));  // inner right edge should be contained

    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, 3)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(
        contains(intersecting_poly, Point(0, 2)));  // inner top edge should be contained

    EXPECT_FALSE(contains(intersecting_poly, Point(3.0f, 0.0f)));
    EXPECT_FALSE(contains(intersecting_poly, Point(-3.0f, 0.0f)));

    EXPECT_FALSE(contains(intersecting_poly, Point()));
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, 1.9)));  // in the hole of the polygon, not contained
    EXPECT_FALSE(contains(intersecting_poly,
                          Point(0, 1.5)));  // in the hole of the polygon, not contained
    EXPECT_FALSE(contains(intersecting_poly, Point(3, 0)));

    EXPECT_TRUE(contains(intersecting_poly, Point(-2, 2)));
    EXPECT_TRUE(contains(intersecting_poly, Point(-2.5, 2)));
    EXPECT_TRUE(contains(intersecting_poly, Point(2.5, 2)));
    EXPECT_TRUE(contains(intersecting_poly, Point(-3, 2)));
}

// All of the below are what's known as "white box tests". That means these tests are
// written with knowledge of how the function is implemented, to test certain internal
// edge cases. These tests are written with the knowledge that the
// 'Polygon::contains(Point)' function uses a ray that is shot in the +x direction
TEST(
    ContainsTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 1);

    bool result = contains(polygon, point);
    EXPECT_FALSE(result);
}

TEST(
    ContainsTest,
    test_polygon_triangle_contains_point_with_point_on_edge_of_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(0.5, 0);

    bool result = contains(polygon, point);
    EXPECT_TRUE(result);
}

TEST(
    ContainsTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 0);

    bool result = contains(polygon, point);
    EXPECT_FALSE(result);
}

TEST(ContainsTest, test_polygon_triangle_contains_point_with_point_on_side)
{
    Polygon polygon({Point(1, 0), Point(0, 1), Point(0, -1)});
    Point point(0.25, 0);

    bool result = contains(polygon, point);
    EXPECT_TRUE(result);
}

TEST(ContainsTest, test_point_in_different_quadrant)
{
    EXPECT_FALSE(contains(Rectangle(Point(0, 0), Point(-2, -2)), Point(1, 1)));
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(-2, -2)), Point(-1, -1)));
}

TEST(ContainsTest, test_point_in_same_quadrant)
{
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(3, 3)), Point(1, 2)));
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-2, 3)));
}

TEST(ContainsTest, test_point_on_rectangle_corner)
{
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(2, 2)), Point(0, 0)));
}

TEST(ContainsTest, test_point_on_rectangle_edge)
{
    EXPECT_TRUE(contains(Rectangle(Point(0, 0), Point(3, 3)), Point(3, 3)));
}

TEST(ContainsTest, test_point_off_left_of_rectangle)
{
    EXPECT_FALSE(contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-7, 2)));
}

TEST(ContainsTest, test_point_off_right_of_rectangle)
{
    EXPECT_FALSE(contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(1, 0)));
}

TEST(ContainsTest, test_point_off_below_rectangle)
{
    EXPECT_FALSE(contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-2, -1)));
}

TEST(ContainsTest, test_point_off_above_rectangle)
{
    EXPECT_FALSE(contains(Rectangle(Point(0, 0), Point(-4, 4)), Point(-2, 5)));
}

TEST(ContainsTest, test_point_centre_of_rectangle)
{
    EXPECT_TRUE(contains(Rectangle(Point(1, 1), Point(-1, -1)), Point(0.5, 0.5)));
}

TEST(ContainsTest, test_ray_contains_point_no_x_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, 0.5);

    EXPECT_TRUE(contains(ray, point));
}

TEST(ContainsTest, test_ray_doesnt_contain_point_behind_start)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(0, -0.5);

    EXPECT_FALSE(contains(ray, point));
}

TEST(ContainsTest, test_ray_doesnt_contain_point)
{
    Ray ray     = Ray(Point(0, 0), Vector(0, 1));
    Point point = Point(-5, 10);

    EXPECT_FALSE(contains(ray, point));
}

TEST(ContainsTest, test_ray_contains_point_no_y_deviation)
{
    Ray ray     = Ray(Point(0, 0), Vector(1, 0));
    Point point = Point(0.5, 0);

    EXPECT_TRUE(contains(ray, point));
}

TEST(ContainsTest, test_diagonal_ray_contains_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(1, 1);

    EXPECT_TRUE(contains(ray, point));
}

TEST(ContainsTest, test_ray_contains_distant_point)
{
    Ray ray     = Ray(Point(2, 2), Vector(-1, -1));
    Point point = Point(-20, -20);

    EXPECT_TRUE(contains(ray, point));
}

TEST(ContainsTest, test_ray_contains_ray_start)
{
    Ray ray = Ray(Point(2, 2), Vector(-1, -1));

    EXPECT_TRUE(contains(ray, ray.getStart()));
}

TEST(ContainsTest, test_segment_contains_point_no_x_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(0, 1));
    Point point     = Point(0, 0.5);

    EXPECT_TRUE(contains(segment, point));
}

TEST(ContainsTest, test_segment_contains_point_no_y_deviation)
{
    Segment segment = Segment(Point(0, 0), Point(1, 0));
    Point point     = Point(0.5, 0);

    EXPECT_TRUE(contains(segment, point));
}

TEST(ContainsTest, test_diagonal_segment_contains_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(1, 1);

    EXPECT_TRUE(contains(segment, point));
}

TEST(ContainsTest, test_segment_doesnt_contain_point)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));
    Point point     = Point(-2, -2);

    EXPECT_FALSE(contains(segment, point));
}

TEST(ContainsTest, test_segment_contains_endpoints)
{
    Segment segment = Segment(Point(2, 2), Point(-1, -1));

    EXPECT_TRUE(contains(segment, segment.getSegStart()));
    EXPECT_TRUE(contains(segment, segment.getEnd()));
}

TEST(ContainsTest, vertical_segment_contains_point)
{
    // A point that is very close to being collinear with a vertical segment
    Segment segment(Point(202, 15), Point(202, -15));
    Point point(202.00000000000003, -0.5);

    EXPECT_TRUE(contains(segment, point));
}
