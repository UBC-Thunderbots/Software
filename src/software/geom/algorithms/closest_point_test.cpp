#include "software/geom/algorithms/closest_point.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(ClosestPointTest, point_on_line)
{
    Point p(4, 4);
    Line l(Point(0, 0), Point(1, 1));
    Point expected(4, 4);
    EXPECT_EQ(closestPoint(p, l), expected);
    EXPECT_EQ(closestPoint(l, p), expected);
}

TEST(ClosestPointTest, point_near_vertical_line)
{
    Point p(5, 5);
    Line l(Point(2, 0), Point(2, 2));
    Point expected(2, 5);
    EXPECT_EQ(closestPoint(p, l), expected);
    EXPECT_EQ(closestPoint(l, p), expected);
}

TEST(ClosestPointTest, point_near_negative_slope_line)
{
    Point p(0, 0);
    Line l(Point(2, 0), Point(0, 2));
    Point expected(1, 1);
    EXPECT_EQ(closestPoint(p, l), expected);
    EXPECT_EQ(closestPoint(l, p), expected);
}

TEST(ClosestPointTest, point_near_positive_slope_line)
{
    Point p(-2, 2.5);
    Line l(Point(1, 0), Point(2, 2));
    Point expected(1.4, 0.8);
    EXPECT_EQ(closestPoint(p, l), expected);
    EXPECT_EQ(closestPoint(l, p), expected);
}

TEST(ClosestPointTest, point_far_from_line)
{
    Point p(-1200000, 400000.5);
    Line l(Point(2, 4.5), Point(0.5, 0));
    Point expected(0.6, 0.3);
    EXPECT_EQ(closestPoint(p, l), expected);
    EXPECT_EQ(closestPoint(l, p), expected);
}

TEST(GeomUtilTest, test_closest_lineseg_intermediate_point)
{
    Segment seg(Point{-2, 1}, Point{1, 2});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(1, 0), seg),
                                               Point(0.4, 1.8), 0.00001));
}

TEST(GeomUtilTest, test_closest_lineseg_point_on_seg)
{
    Segment seg(Point{-2, 1}, Point{1, 2});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(-1.4, 1.2), seg),
                                               Point(-1.4, 1.2), 0.00001));
}

TEST(GeomUtilTest, test_closest_lineseg_middle_point)
{
    Segment seg(Point{-1, 1}, Point{1, 1});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0, 2), seg),
                                               Point(0, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_lineseg_start_point)
{
    Segment seg(Point{-1, 1}, Point{1, 1});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(-2, 1.5), seg),
                                               Point(-1, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_lineseg_end_point)
{
    Segment seg(Point{-1, 1}, Point{1, 1});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(2, 1.5), seg),
                                               Point(1, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_lineseg_degenerate)
{
    Segment seg(Point{1, 1}, Point{1, 1});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(1, 0), seg),
                                               Point(1, 1), 0.00001));
}


TEST(GeomUtilTest, test_closest_circle_inside)
{
    Circle circle(Point(0, 0), 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0, 0.5), circle),
                                               Point(0, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_circle_outside)
{
    Circle circle(Point(1, 1), 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(1, 3), circle),
                                               Point(1, 2), 0.00001));
}

TEST(GeomUtilTest, test_closest_circle_point_on_edge)
{
    Circle circle(Point(1, 1), 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(2, 1), circle),
                                               Point(2, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_inside)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0.9, 0.11), polygon),
                                               Point(1, 0.11), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_point_on_edge)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0.5, 1), polygon),
                                               Point(0.5, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_outside)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(-0.1, 0.5), polygon),
                                               Point(0, 0.5), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_outside_end_point)
{
    Polygon polygon({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(1.5, 1.5), polygon),
                                               Point(1, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_concave_outside)
{
    Polygon polygon(
        {Point(0, 0), Point(1, 0), Point(1, 1), Point(0.5, 0.5), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0.1, 1.5), polygon),
                                               Point(0, 1), 0.00001));
}

TEST(GeomUtilTest, test_closest_polygon_concave_inside)
{
    Polygon polygon(
        {Point(0, 0), Point(1, 0), Point(1, 1), Point(0.5, 0.85), Point(0, 1)});

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(0.5, 0.8), polygon),
                                               Point(0.5, 0.85), 0.00001));
}
TEST(GeomUtilTest, test_closest_stadium_outside_semicircle)
{
    Stadium stadium(Point(0, 0), Point(2, 0), 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(4, 0), stadium), Point(3, 0), 0.00001));
}

TEST(GeomUtilTest, test_closest_stadium_outside_rectangle)
{
    Stadium stadium(Point(0, 0), Point(2, 0), 1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(closestPoint(Point(1, 2), stadium), Point(1, 1), 0.00001));
}
