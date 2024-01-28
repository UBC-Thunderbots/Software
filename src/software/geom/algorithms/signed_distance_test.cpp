#include "software/geom/algorithms/signed_distance.h"

#include <gtest/gtest.h>

#include "software/geom/line.h"

// ------------------------------------------------------------------------------------------------
// CIRCLE TESTS
// ------------------------------------------------------------------------------------------------

TEST(SignedDistanceTest, point_middle_circle)
{
    Circle circle(Point(2,-3), 5);
    Point point(2,-3);
    double expected = -5;
    EXPECT_DOUBLE_EQ(signedDistance(circle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, circle), expected);
}

TEST(SignedDistanceTest, point_perimeter_circle)
{
    Circle circle(Point(2,-3), 5);
    Point point(2,2);
    double expected = 0;
    EXPECT_DOUBLE_EQ(signedDistance(circle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, circle), expected);
}

TEST(SignedDistanceTest, diagonal_point_perimeter_circle)
{
    Circle circle(Point(-2,1), 2);
    Point point = Point(-2 + 2*cos(M_PI/6),1+2*sin(M_PI/6));
    double expected = 0;
    EXPECT_DOUBLE_EQ(signedDistance(circle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, circle), expected);
}

TEST(SignedDistanceTest, point_outside_circle)
{
    Circle circle(Point(2,-3), 4);
    Point point(2,4);
    double expected = 3;
    EXPECT_DOUBLE_EQ(signedDistance(circle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, circle), expected);
}

TEST(SignedDistanceTest, diagonal_point_outside_circle)
{
    Circle circle(Point(2,-5), 3);
    Point point = Point(2,-5) + Vector(2,3).normalize(6);
    double expected = 3;
    EXPECT_DOUBLE_EQ(signedDistance(circle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, circle), expected);
}

// ------------------------------------------------------------------------------------------------
// RECTANGLE TESTS
// ------------------------------------------------------------------------------------------------

TEST(SignedDistanceTest, point_middle_rectangle)
{
    Rectangle rectangle(Point(-3,-3), Point(3, 1));
    Point point(0,-1);
    double expected = -2;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_middle_side_rectangle)
{
    Rectangle rectangle(Point(10,-3), Point(2, 1));
    Point point(4,-1);
    double expected = -2;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_top_perimeter_rectangle)
{
    Rectangle rectangle(Point(3,0), Point(5, 2));
    Point point(4,2);
    double expected = 0;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_side_perimeter_rectangle)
{
    Rectangle rectangle(Point(3,0), Point(5, 2));
    Point point(3,1);
    double expected = 0;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_corner_perimeter_rectangle)
{
    Rectangle rectangle(Point(3,0), Point(5, 2));
    Point point(5,0);
    double expected = 0;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_outside_horizontal_rectangle)
{
    Rectangle rectangle(Point(3,0), Point(5, 2));
    Point point(7,0);
    double expected = 2;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}

TEST(SignedDistanceTest, point_outside_diagonal_rectangle)
{
    Rectangle rectangle(Point(3,0), Point(5, 2));
    Point point(5+cos(M_PI/3),0-sin(M_PI/3));
    double expected = 1;
    EXPECT_DOUBLE_EQ(signedDistance(rectangle, point), expected);
    EXPECT_DOUBLE_EQ(signedDistance(point, rectangle), expected);
}
// ------------------------------------------------------------------------------------------------
// POLYGON TESTS
// ------------------------------------------------------------------------------------------------

TEST(SignedDistanceTest, point_vertical_polygon)
{
    Polygon polygon({Point(0,0), Point(3,0), Point(5,-7), Point(-1, -9), Point(-2, -8.5)});
    Point point0(1.5,-1.5);
    double expected0 = -1.5;

    Point point1(2,0);
    double expected1 = 0;

    Point point2(2,2);
    double expected2 = 2;

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, polygon), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, polygon), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, polygon), expected2);
}

TEST(SignedDistanceTest, point_horizontal_polygon)
{
    Polygon polygon({Point(0,0), Point(3,0), Point(3,-7), Point(-1, -9)});

    Point point0(1.7,-4);
    double expected0 = -1.3;

    Point point1(3,-4);
    double expected1 = 0;

    Point point2(6,-5);
    double expected2 = 3;

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, polygon), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, polygon), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, polygon), expected2);
}

TEST(SignedDistanceTest, point_diagonal_polygon)
{
    Polygon polygon({Point(0,0), Point(3,0), Point(6,3), Point(-1, 10), Point(1, 7)});

    Point point0(4,1);
    double expected0 = 0;

    Point point1(4-cos(M_PI_4),1+sin(M_PI_4));
    double expected1 = -1;

    Point point2(4+cos(M_PI_4)*2,1-sin(M_PI_4)*2);
    double expected2 = 2;

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, polygon), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, polygon), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(polygon, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, polygon), expected2);
}

// ------------------------------------------------------------------------------------------------
// STADIUM TESTS
// ------------------------------------------------------------------------------------------------

TEST(SignedDistanceTest, point_vertical_stadium)
{
    Stadium stadium(Point(-2,1), Point(6, 1), 1.5);
    Point point0(0,1);
    double expected0 = -1.5;

    Point point1(1.5,.5);
    double expected1 = -1;

    Point point2(2,3);
    double expected2 = .5;

    Point point3(6,2.5);
    double expected3 = 0;

    Point point4(-1,-6);
    double expected4 = 5.5;

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, stadium), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, stadium), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, stadium), expected2);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point3), expected3);
    EXPECT_DOUBLE_EQ(signedDistance(point3, stadium), expected3);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point4), expected4);
    EXPECT_DOUBLE_EQ(signedDistance(point4, stadium), expected4);
}

TEST(SignedDistanceTest, point_horizontal_stadium)
{
    Stadium stadium(Point(3,0), Point(-2, 0), 2);

    Point point0(3,0);
    double expected0 = -2;

    Point point1(-2,0);
    double expected1 = -2;

    Point point2(-2.5,0);
    double expected2 = -1.5;

    Point point3(-4,0);
    double expected3 = 0;

    Point point4(-9,0);
    double expected4 = 5;

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, stadium), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, stadium), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, stadium), expected2);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point3), expected3);
    EXPECT_DOUBLE_EQ(signedDistance(point3, stadium), expected3);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point4), expected4);
    EXPECT_DOUBLE_EQ(signedDistance(point4, stadium), expected4);
}

TEST(SignedDistanceTest, point_diagonal_stadium)
{
    Stadium stadium(Point(-6,-2), Point(6, -1), 3);
    Point point0(6+cos(M_PI/6)*2,-1-sin(M_PI/6)*2);
    double expected0 = -1;

    Point point1(6+cos(M_PI/3)*2.5,-1-sin(M_PI/3)*2.5);
    double expected1 = -.5;

    Point point2(6+cos(M_PI/6)*3,-1+sin(M_PI/6)*3);
    double expected2 = 0;

    Point point3(6+cos(M_PI/6)*5,-1-sin(M_PI/6)*5);
    double expected3 = 2;

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point0), expected0);
    EXPECT_DOUBLE_EQ(signedDistance(point0, stadium), expected0);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point1), expected1);
    EXPECT_DOUBLE_EQ(signedDistance(point1, stadium), expected1);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point2), expected2);
    EXPECT_DOUBLE_EQ(signedDistance(point2, stadium), expected2);

    EXPECT_DOUBLE_EQ(signedDistance(stadium, point3), expected3);
    EXPECT_DOUBLE_EQ(signedDistance(point3, stadium), expected3);
}
