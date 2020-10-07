#include "software/geom/convex_polygon.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "software/geom/point.h"
#include "software/test_util/test_util.h"

TEST(ConvexPolygonConstructorTest, test_construct_from_vector)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    std::vector<Point> points{p1, p2, p3};
    ConvexPolygon poly(points);

    // check that all of the points are in the convex polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : poly.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the correct segments are in the convex polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : poly.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}

TEST(ConvexPolygonConstructorTest, test_construct_from_initializer_list)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    ConvexPolygon poly{p1, p2, p3};

    // check that all of the points are in the convex polygon
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

TEST(ConvexPolygonConstructorTest, test_not_convex)
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
    EXPECT_THROW(ConvexPolygon({{0.0f, 0.0f},
                                {0.0f, 2.0f},
                                {2.0f, 2.0f},
                                {2.0f, 1.0f},
                                {0.0f, 1.0f},
                                {-0.0f, -2.0f},
                                {-2.0f, -2.0f},
                                {-2.0f, -1.0f},
                                {0.0f, -1.0f}}),
                 std::invalid_argument);
}

TEST(ConvexPolygonConstructorTest, test_self_intersecting_loop)
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
    EXPECT_THROW(ConvexPolygon({{-3.0f, 0.0f},
                                {-3.0f, 3.0f},
                                {3.0f, 3.0f},
                                {3.0f, 0.0f},
                                {-2.0f, 2.0f},
                                {2.0f, 2.0f}}),
                 std::invalid_argument);
}

TEST(ConvexPolygonConstructorTest, test_ribbon_not_convex)
{
    EXPECT_THROW(ConvexPolygon({{0, 0}, {0, 5}, {5, 5}, {-5, 0.0f}}),
                 std::invalid_argument);
}

TEST(ConvexPolygonAreaTest, test_trapezoid_area)
{
    ConvexPolygon trapezoid = ConvexPolygon{{0, 0}, {1, 4}, {5, 4}, {6, 0}};
    EXPECT_EQ(trapezoid.area(), 20);
}

TEST(ConvexPolygonAreaTest, test_rhombus_area)
{
    ConvexPolygon rhombus = ConvexPolygon{{0, 0}, {5, 3}, {10, 0}, {5, -7}};
    EXPECT_EQ(rhombus.area(), 50);
}

TEST(ConvexPolygonIsConvexTest, test_barely_convex_polygon)
{
    EXPECT_NO_THROW(ConvexPolygon({{0, 0}, {4.5, 0.5}, {4.5, -0.5}}));
}

TEST(ConvexPolygonIsConvexTest, test_degenerate_polygon)
{
    EXPECT_THROW(
        ConvexPolygon({Point(2, 3), Point(2, 3), Point(2, 3), Point(2, 3), Point(2, 3)}),
        std::invalid_argument);
}

TEST(ConvexPolygonIsConvexTest, test_degenerate_polygon_linear_points)
{
    EXPECT_THROW(
        ConvexPolygon({Point(0, 3), Point(1, 3), Point(2, 3), Point(3, 3), Point(4, 3)}),
        std::invalid_argument);
}
