#include "software/geom/algorithms/rasterize.h"
#include "software/geom/algorithms/contains.h"

#include <gtest/gtest.h>


//////////////////////////////////////////////////////
////              Testing Circles                 ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, segment_in_circle)
{
    Circle c({1.0, 1.0}, 4.0);
    Segment s({-2, 2}, {4, 0});
    EXPECT_TRUE(contains(c, s));
}

//////////////////////////////////////////////////////
////              Testing Polygons                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_polygon_triangle_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{0.9f, 0.9f};
    Polygon triangle{p1, p2, p3};
    EXPECT_TRUE(contains(triangle, point));
}

TEST(RasterizeTest, test_polygon_triangle_not_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};
    EXPECT_FALSE(contains(triangle, point));
}

TEST(RasterizeTest, test_polygon_hexagon_contains_point)
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

TEST(RasterizeTest, test_self_intersecting_polygon_contains)
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

TEST(RasterizeTest, test_complex_self_intersecting_polygon_contains)
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

TEST(RasterizeTest, test_self_intersecting_loop_polygon_contains)
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
        RasterizeTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 1);

    bool result = contains(polygon, point);
    EXPECT_FALSE(result);
}

TEST(
        RasterizeTest,
    test_polygon_triangle_contains_point_with_point_on_edge_of_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(0.5, 0);

    bool result = contains(polygon, point);
    EXPECT_TRUE(result);
}

TEST(
    RasterizeTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 0);

    bool result = contains(polygon, point);
    EXPECT_FALSE(result);
}

TEST(RasterizeTest, test_polygon_triangle_contains_point_with_point_on_side)
{
    Polygon polygon({Point(1, 0), Point(0, 1), Point(0, -1)});
    Point point(0.25, 0);

    bool result = contains(polygon, point);
    EXPECT_TRUE(result);
}

//////////////////////////////////////////////////////
////            Testing Rectangles                ////
//////////////////////////////////////////////////////
TEST(RasterizeTest, test_offset_)
{
    // offset will las
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double offset = 0.5f;
    std::vector<Point> rasterized_points = rasterize(rectangle, offset);

    for(Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    // TODO test the size of the array being correct aswell (can't test that in this case as we don't know 0.5 + 0.5 > or < 1??)
}

TEST(RasterizeTest, test_offset_larger_than_rectangle)
{
    Rectangle rectangle(Point(0, 0), Point(1, 1));
    double offset = 2;
    std::vector<Point> rasterized_points = rasterize(rectangle, offset);

    for(Point p : rasterized_points)
    {
        EXPECT_TRUE(contains(rectangle, p));
    }
    EXPECT_EQ(rasterized_points.size(), 1);
}