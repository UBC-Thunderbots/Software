#include <gtest/gtest.h>

#include <unordered_set>

#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"

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
    Polygon hexagon{{0.0f, 2.0f},     // top vertex
                    {2.0f, 1.0f},     // top right vertex
                    {2.0f, -1.0f},    // bottom right vertex
                    {0.0f, -2.0f},    // bottom vertex
                    {-2.0f, -1.0f},   // bottom left vertex
                    {-2.0f, 1.0f}};   // top left vertex

    EXPECT_TRUE(hexagon.contains(Point()));
    EXPECT_FALSE(hexagon.contains(Point(0, 2.01)));
    EXPECT_FALSE(hexagon.contains(Point(0, -2.01)));
    EXPECT_FALSE(hexagon.contains(Point(2.01, 0)));
    EXPECT_FALSE(hexagon.contains(Point(-2.01, 0)));
    EXPECT_FALSE(hexagon.contains(Point(2.0f, 0.0f)));  // on right edge, see NOTE on Polygon::contains
    EXPECT_TRUE(hexagon.contains(Point(-2, 0)));        // on left edge
    EXPECT_TRUE(hexagon.contains(Point(-2, -1)));       // the bottom left vertex of the hexagon
    EXPECT_TRUE(hexagon.contains(Point(1, -1)));
    EXPECT_TRUE(hexagon.contains(Point(-1.5, 0.75)));
}

// All of the below are what's known as "white box tests". That means these tests are
// written with knowledge of how the function is implemented, to test certain internal
// edge cases. These tests are written with the knowledge that the 'Polygon::contains(Point)'
// function uses a ray that is shot in the +x direction
TEST(PolygonTest,
     test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 1);

    bool result = polygon.contains(point);
    EXPECT_FALSE(result);
}

TEST(PolygonTest,
     test_polygon_triangle_contains_point_with_point_on_edge_of_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(0.5, 0);

    bool result = polygon.contains(point);
    EXPECT_TRUE(result);
}

TEST(PolygonTest,
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
