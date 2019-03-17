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

TEST(PolygonTest, test_polygon_triangle_intersects_line_segment)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 0.0f};
    Point point{2.0f, 2.0f};
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

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
