#include "geom/polygon.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "geom/point.h"

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

    // check that the segments are in the polygon
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

    // check that the segments are in the polygon
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
    EXPECT_TRUE(triangle.containsPoint(point));
}

TEST(PolygonTest, test_polygon_triangle_not_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};
    EXPECT_FALSE(triangle.containsPoint(point));
}

TEST(PolygonTest, test_polygon_hexagon_contains_point)
{
    Point p1{-1.499421, 1.327846}, p2{-1.679421, 1.223923}, p3{-1.679421, 1.016077},
        p4{-1.499421, 0.912154}, p5{-1.319421, 1.016077}, p6{-1.319421, 1.223923};

    Point point;
    Polygon hexagon{p1, p2, p3, p4, p5, p6};
    point = Point{-1.050000, 0.090000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.050000, 0.180000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.230000, 0.360000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.140000, 0.450000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.320000, 0.540000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.410000, 0.630000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.410000, 0.720000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.500000, 0.810000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.410000, 0.810000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.320000, 0.810000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.230000, 0.810000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.320000, 0.900000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.410000, 0.990000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.320000, 0.990000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.500000, 1.080000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.410000, 1.080000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.320000, 1.080000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.500000, 1.170000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.410000, 1.170000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.410000, 1.170000};
    EXPECT_TRUE(hexagon.containsPoint(point));
    point = Point{-1.770000, 1.260000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.770000, 1.170000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.770000, 1.080000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.770000, 0.990000};
    EXPECT_FALSE(hexagon.containsPoint(point));
    point = Point{-1.680000, 0.990000};
    EXPECT_FALSE(hexagon.containsPoint(point));
}


TEST(PolygonTest, test_polygon_triangle_intersects_line_segment)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{0.0f, 1.0f};
    Polygon triangle{p1, p2, p3};

    Segment seg{Point(1.0f, 1.0f), Point(0.2f, 0.2f)};
    EXPECT_TRUE(triangle.intersects(seg));
}

TEST(PolygonTest, test_polygon_triangle_not_intersects_line_segment)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 0.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};

    Segment seg{Point(1.0f, 1.0f), Point(2.0f, 2.0f)};
    EXPECT_FALSE(triangle.intersects(seg));
}

TEST(PolygonTest, test_polygon_triangle_intersects_ray)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 0.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};

    Ray ray{Point(5000.0f, 5000.0f), Vector(-1.0, -1.0)};
    EXPECT_TRUE(triangle.intersects(ray));
}

TEST(PolygonTest, test_polygon_triangle_not_intersects_ray)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 0.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};

    Ray ray{Point(5000.0f, 5000.0f), Vector(1.0f, 1.0f)};
    EXPECT_FALSE(triangle.intersects(ray));
}

TEST(PolygonTest, test_polygon_triangle_intersects_ray2)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 0.0f};
    Point point{2.0f, 2.0f};
    Polygon triangle{p1, p2, p3};

    Ray ray{Point(-0.1f, -0.1f), Vector(-1.0f, -1.0f)};
    EXPECT_FALSE(triangle.intersects(ray));
}

// All of the below are what's known as "white box tests". That means these tests are
// written with knowledge of how the function is implemented, to test certain internal
// edge cases. These tests are written with the knowledge that the 'containsPoint'
// function uses a ray that is shot in the +x direction
TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 1);

    bool result = polygon.containsPoint(point);
    EXPECT_FALSE(result);
}

TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_on_edge_of_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(0.5, 0);

    bool result = polygon.containsPoint(point);
    EXPECT_TRUE(result);
}

TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_inside_triangle_and_ray_intersecting_vertex)
{
    Polygon polygon({Point(1, 0), Point(0, 1), Point(0, -1)});
    Point point(0.25, 0);

    bool result = polygon.containsPoint(point);
    EXPECT_TRUE(result);
}

TEST(
    PolygonTest,
    test_polygon_triangle_contains_point_with_point_outside_triangle_and_ray_overlapping_segment)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(0, 1)});
    Point point(-1, 0);

    bool result = polygon.containsPoint(point);
    EXPECT_FALSE(result);
}

TEST(PolygonTest, test_polygon_triangle_contains_point_with_point_on_vertex)
{
    Polygon polygon({Point(1, 0), Point(0, 1), Point(0, -1)});
    Point point(0.25, 0);

    bool result = polygon.containsPoint(point);
    EXPECT_TRUE(result);
}

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
