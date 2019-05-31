#include "geom/triangle.h"

#include <gtest/gtest.h>

#include <unordered_set>

#include "geom/point.h"
#include "geom/polygon.h"

TEST(TriangleTest, test_construct_from_points)
{
    Point p1{0.0f, 0.0f}, p2{2.0f, 0.0f}, p3{0.0f, 2.0f};
    Triangle triangle(p1, p2, p3);

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : triangle.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : triangle.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}

TEST(TriangleTest, test_construct_from_vector)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 2.0f};
    std::vector<Point> points{p1, p2, p3};
    Triangle triangle(points);

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : triangle.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : triangle.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}

TEST(TriangleTest, test_construct_from_initializer_list)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 1.0f}, p3{1.0f, 2.0f};
    Triangle triangle{p1, p2, p3};

    // check that all of the points are in the polygon
    std::unordered_set<Point> points_set{p1, p2, p3};
    for (const auto& p : triangle.getPoints())
    {
        EXPECT_TRUE(points_set.find(p) != points_set.end());
    }

    // check that the segments are in the polygon
    std::unordered_set<Segment> segments_set = {Segment{p1, p2}, Segment{p2, p3},
                                                Segment{p3, p1}};
    for (const auto& seg : triangle.getSegments())
    {
        EXPECT_TRUE(segments_set.find(seg) != segments_set.end());
    }
}


TEST(TriangleTest, test_triangle_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{0.9f, 0.9f};
    Triangle triangle{p1, p2, p3};
    EXPECT_TRUE(triangle.containsPoint(point));
}

TEST(TriangleTest, test_triangle_contains_point_corner)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{1.0f, 1.0f};
    Triangle triangle{p1, p2, p3};
    EXPECT_FALSE(triangle.containsPoint(point));
}

TEST(TriangleTest, test_triangle_not_contains_point)
{
    Point p1{0.0f, 0.0f}, p2{1.0f, 0.0f}, p3{1.0f, 1.0f};
    Point point{2.0f, 2.0f};
    Triangle triangle{p1, p2, p3};
    EXPECT_FALSE(triangle.containsPoint(point));
}

TEST(TriangleTest, test_triangle_intersects_line_segment)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Segment seg{Point(1.0f, 1.0f), Point(0.2f, 0.2f)};
    EXPECT_TRUE(triangle.intersects(seg));
}

TEST(TriangleTest, test_triangle_not_intersects_line_segment)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Segment seg{Point(1.0f, 1.0f), Point(2.0f, 2.0f)};
    EXPECT_FALSE(triangle.intersects(seg));
}

TEST(TriangleTest, test_triangle_intersects_two_segments_outside)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Segment seg{Point(0.0f, -1.0f), Point(2.0f, 1.0f)};
    EXPECT_TRUE(triangle.intersects(seg));
}

TEST(TriangleTest, test_triangle_intersects_two_segments_inside)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Segment seg{Point(-0.1f, -0.1f), Point(0.1f, 0.1f)};
    EXPECT_TRUE(triangle.intersects(seg));
}

TEST(TriangleTest, test_triangle_intersects_ray)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(5000.0f, 5000.0f), Vector(-1.0, -1.0)};
    EXPECT_TRUE(triangle.intersects(ray));
}

TEST(TriangleTest, test_triangle_not_intersects_ray)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(5000.0f, 5000.0f), Vector(1.0f, 1.0f)};
    EXPECT_FALSE(triangle.intersects(ray));
}

TEST(TriangleTest, test_triangle_intersects_ray2)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(-0.1f, -0.1f), Vector(-1.0f, -1.0f)};
    EXPECT_FALSE(triangle.intersects(ray));
}

TEST(TriangleTest, test_triangle_intersects_ray_corner)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(0.0f, -1.0f), Vector(1.0f, 1.0f)};
    EXPECT_TRUE(triangle.intersects(ray));
}

TEST(TriangleTest, test_triangle_intersects_ray_corner_out)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(-0.1f, -0.1f), Vector(1.0f, 1.0f)};
    EXPECT_TRUE(triangle.intersects(ray));
}

TEST(TriangleTest, test_triangle_intersects_ray_corner_in)
{
    Point p1{0.0f, 0.0f}, p2{0.0f, 1.0f}, p3{1.0f, 0.0f};
    Triangle triangle{p1, p2, p3};

    Ray ray{Point(0.1f, 0.1f), Vector(-1.0f, -1.0f)};
    EXPECT_TRUE(triangle.intersects(ray));
}

int main(int argc, char** argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
