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
