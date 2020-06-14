#include "software/new_geom/polygon.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "software/new_geom/point.h"
#include "software/test_util/test_util.h"

TEST(PolygonTest, test_construct_from_vector)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    std::vector<Point> points{p1, p2, p3};
    Polygon poly(points);

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : poly.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the correct segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : poly.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}

TEST(PolygonTest, test_construct_from_initializer_list)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Polygon poly{p1, p2, p3};

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : poly.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the correct segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : poly.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}


TEST(PolygonTest, test_polygon_triangle_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{0.9f, 0.9f};
    Polygon triangle{p1, p2, p3};
    EXPECT_TRUE(triangle.contains(point));
}

TEST(PolygonTest, test_polygon_triangle_not_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};
    EXPECT_FALSE(triangle.contains(point));
}

TEST(PolygonTest, test_polygon_hexagon_contains_point)
{
    // Hexagon centered at origin with the following points
    Polygon hexagon{{0.0f, 2.0f},    // top vertex
                    {2.0f, 1.0f},    // top right vertex
                    {2.0f, -1.0f},   // bottom right vertex
                    {0.0f, -2.0f},   // bottom vertex
                    {-2.0f, -1.0f},  // bottom left vertex
                    {-2.0f, 1.0f}};  // top left vertex

    EXPECT_TRUE(hexagon.contains(Point()));
    EXPECT_FALSE(hexagon.contains(Point(0, 2.01)));
    EXPECT_FALSE(hexagon.contains(Point(0, -2.01)));
    EXPECT_FALSE(hexagon.contains(Point(2.01, 0)));
    EXPECT_FALSE(hexagon.contains(Point(-2.01, 0)));
    EXPECT_FALSE(hexagon.contains(
        Point(2.0f, 0.0f)));  // on right edge, see NOTE on Polygon::contains
    EXPECT_TRUE(hexagon.contains(Point(-2, 0)));  // on left edge
    EXPECT_TRUE(
        hexagon.contains(Point(-2, -1)));  // the bottom left vertex of the hexagon
    EXPECT_TRUE(hexagon.contains(Point(1, -1)));
    EXPECT_TRUE(hexagon.contains(Point(-1.5, 0.75)));
}

TEST(PolygonTest, test_self_intersecting_polygon_contains)
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

    EXPECT_FALSE(intersecting_poly.contains(
        Point()));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(Point(2, 0)));
    EXPECT_FALSE(intersecting_poly.contains(Point(0.5, 0.5)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 0.5)));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_TRUE(intersecting_poly.contains(Point(0, 1)));
    EXPECT_TRUE(intersecting_poly.contains(Point(1, 1.5)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(intersecting_poly.contains(Point(1, 1)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(1, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(intersecting_poly.contains(Point(0, 1.5)));
}

TEST(PolygonTest, test_complex_self_intersecting_polygon_contains)
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

    EXPECT_FALSE(intersecting_poly.contains(
        Point()));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(Point(2, 0)));
    EXPECT_FALSE(intersecting_poly.contains(Point(0.5, 0.5)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 0.5)));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(
        Point(1, 2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(Point(-2, 0)));
    EXPECT_FALSE(intersecting_poly.contains(Point(-0.5, -0.5)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, -0.5)));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, -2)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(Point(0, -1)));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(-1, -1)));  // on a "top" edge, see NOTE on Polygon::contains
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, -1.5)));  // on a "right" edge, see NOTE on Polygon::contains
    EXPECT_TRUE(intersecting_poly.contains(Point(-1, -2)));
    EXPECT_TRUE(intersecting_poly.contains(Point(0, 1)));
    EXPECT_TRUE(intersecting_poly.contains(Point(1, 1.5)));
    EXPECT_TRUE(intersecting_poly.contains(Point(1, 1)));
    EXPECT_TRUE(intersecting_poly.contains(Point(0, 1.5)));
    EXPECT_TRUE(intersecting_poly.contains(Point(-1, -1.5)));
}

TEST(PolygonTest, test_self_intersecting_loop_polygon_contains)
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

    EXPECT_FALSE(intersecting_poly.contains(
        Point(3, 2)));  // on a right edge, see NOTE on Polygon::contains
    EXPECT_TRUE(
        intersecting_poly.contains(Point(2, 2)));  // inner right edge should be contained

    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 3)));  // on a top edge, see NOTE on Polygon::contains
    EXPECT_TRUE(
        intersecting_poly.contains(Point(0, 2)));  // inner top edge should be contained

    EXPECT_FALSE(intersecting_poly.contains(Point(3.0f, 0.0f)));
    EXPECT_FALSE(intersecting_poly.contains(Point(-3.0f, 0.0f)));

    EXPECT_FALSE(intersecting_poly.contains(Point()));
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 1.9)));  // in the hole of the polygon, not contained
    EXPECT_FALSE(intersecting_poly.contains(
        Point(0, 1.5)));  // in the hole of the polygon, not contained
    EXPECT_FALSE(intersecting_poly.contains(Point(3, 0)));

    EXPECT_TRUE(intersecting_poly.contains(Point(-2, 2)));
    EXPECT_TRUE(intersecting_poly.contains(Point(-2.5, 2)));
    EXPECT_TRUE(intersecting_poly.contains(Point(2.5, 2)));
    EXPECT_TRUE(intersecting_poly.contains(Point(-3, 2)));
}

// All of the below are what's known as "white box tests". That means these tests are
// written with knowledge of how the function is implemented, to test certain internal
// edge cases. These tests are written with the knowledge that the
// 'Polygon::contains(Point)' function uses a ray that is shot in the +x direction
TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 1);

    bool result = polygon.contains(point);
    EXPECT_FALSE(result);
}

TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_on_edge_of_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(0.5, 0);

    bool result = polygon.contains(point);
    EXPECT_TRUE(result);
}

TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 0);

    bool result = polygon.contains(point);
    EXPECT_FALSE(result);
}

TEST(PolygonTest, test_polygon_triangle_contains_point_with_point_on_side)
{
    Polygon polygon({Point(1, 0), Point(0, 1), Point(0, -1)});
    Point point(0.25, 0);

    bool result = polygon.contains(point);
    EXPECT_TRUE(result);
}

TEST(PolygonCentroidTest, test_triangle)
{
    Polygon poly({{1, 2}, {2, 3}, {1, 3}});
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(Point(1.333, 2.666), poly.centroid()));
}

TEST(PolygonCentroidTest, test_rectangle)
{
    Polygon poly({{-1, -1}, {-1, 3}, {5, 3}, {5, -1}});
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(Point(2, 1), poly.centroid()));
}

TEST(PolygonCentroidTest, test_irregular_shape)
{
    Polygon poly({{-1, -1}, {-8, 4}, {-2, 4}, {-2, 2}, {2, 0}});
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(Point(-2.895, 1.842), poly.centroid()));
}

TEST(PolygonCentroidTest, test_non_convex_five_points_up_left)
{
    Polygon poly({{1, 1}, {1, 3}, {2, 2}, {5, 3}, {5, 1}});
    EXPECT_TRUE(::TestUtil::equalWithinTolerance(Point(3.111, 1.778), poly.centroid()));
}


TEST(PolygonExpandTest, test_three_points_up)
{
    Polygon poly({{1, 2}, {2, 3}, {1, 3}});
    Polygon expected({{1, 2}, {2, 6}, {1, 6}});
    Vector expansion_vector({0, 3});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}

TEST(PolygonExpandTest, test_three_points_left)
{
    Polygon poly({{1, 2}, {2, 3}, {1, 3}});
    Polygon expected({{-1, 2}, {2, 3}, {-1, 3}});
    Vector expansion_vector({-2, 0});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}

TEST(PolygonExpandTest, test_five_points_right)
{
    Polygon poly({{1, 1}, {1, 3}, {2, 3}, {5, 3}, {5, 1}});
    Polygon expected({{1, 1}, {1, 3}, {2, 3}, {8, 3}, {8, 1}});
    Vector expansion_vector({3, 0});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}

TEST(PolygonExpandTest, test_five_points_up_left)
{
    Polygon poly({{1, 1}, {1, 3}, {2, 3}, {5, 3}, {5, 1}});
    Polygon expected({{-1, 4}, {-1, 6}, {0, 6}, {5, 3}, {5, 1}});
    Vector expansion_vector({-2, 3});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}

TEST(PolygonExpandTest, test_non_convex_five_points_up_left)
{
    Polygon poly({{1, 1}, {1, 3}, {2, 2}, {5, 3}, {5, 1}});
    Polygon expected({{-1, 4}, {-1, 6}, {0, 5}, {5, 3}, {5, 1}});
    Vector expansion_vector({-2, 3});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}

TEST(PolygonExpandTest, test_four_points_0_vector)
{
    Polygon poly({{1, 1}, {1, 3}, {3, 3}, {5, 3}, {5, 1}});
    Polygon expected(poly);
    Vector expansion_vector({0, 0});
    EXPECT_EQ(poly.expand(expansion_vector), expected);
}
