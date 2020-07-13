#include "software/test_util/equal_within_tolerance.h"

#include <gtest/gtest.h>

#include <algorithm>

TEST(TestUtilsTest, polygon_check_if_equal_within_tolerance)
{
Polygon poly1({Point(2.323, 2.113), Point(4.567, 1.069), Point(9.245, 1.227)});
Polygon poly2({Point(2.324, 2.114), Point(4.568, 1.07), Point(9.246, 1.228)});
Polygon poly3({Point(2.325, 2.115), Point(4.569, 1.071), Point(9.247, 1.229)});
Polygon poly4(
        {Point(2.325, 2.115), Point(4.569, 1.071), Point(9.247, 1.229), Point(5, 5)});
EXPECT_TRUE(
        ::equalWithinTolerance(poly1, poly2, 2 * METERS_PER_MILLIMETER));
EXPECT_FALSE(::equalWithinTolerance(poly1, poly3));
EXPECT_FALSE(::equalWithinTolerance(poly1, poly4));
}

TEST(TestUtilsTest, point_check_if_equal_within_tolerance)
{
Point point1(8.423, 4.913);
Point point2(8.4232391, 4.9139881);
Point point3(8.424, 4.914);
Point point4(8.425, 4.915);
Point point5(5.393, 1.113);
Point point6(5.394, 1.114);
Point point7(9.245, 1.227);
Point point8(9.246, 1.227);

EXPECT_TRUE(
        equalWithinTolerance(point1, point2, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(
        equalWithinTolerance(point1, point3, 2 * METERS_PER_MILLIMETER));
EXPECT_FALSE(equalWithinTolerance(point1, point4));
EXPECT_TRUE(
        equalWithinTolerance(point5, point6, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(
        equalWithinTolerance(point6, point5, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(equalWithinTolerance(point7, point8));
EXPECT_TRUE(equalWithinTolerance(point8, point7));
}

TEST(TestUtilsTest, circle_check_if_equal_within_tolerance)
{
Circle circle1(Point(5.393, 1.113), 6.567);
Circle circle2(Point(5.394, 1.114), 6.568);
Circle circle3(Point(5.395, 1.115), 6.569);
EXPECT_TRUE(
        equalWithinTolerance(circle1, circle2, 2 * METERS_PER_MILLIMETER));
EXPECT_FALSE(equalWithinTolerance(circle1, circle2));
EXPECT_FALSE(equalWithinTolerance(circle1, circle3));
}

TEST(TestUtilsTest, vector_check_if_equal_within_tolerance)
{
Vector vector1(8.423, 4.913);
Vector vector2(8.4232391, 4.9139881);
Vector vector3(8.424, 4.914);
Vector vector4(8.425, 4.915);
Vector vector5(5.393, 1.113);
Vector vector6(5.394, 1.114);
Vector vector7(9.245, 1.227);
Vector vector8(9.246, 1.227);

EXPECT_TRUE(
        equalWithinTolerance(vector1, vector2, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(
        equalWithinTolerance(vector1, vector3, 2 * METERS_PER_MILLIMETER));
EXPECT_FALSE(equalWithinTolerance(vector1, vector4));
EXPECT_TRUE(
        equalWithinTolerance(vector5, vector6, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(
        equalWithinTolerance(vector6, vector5, 2 * METERS_PER_MILLIMETER));
EXPECT_TRUE(equalWithinTolerance(vector7, vector8));
EXPECT_TRUE(equalWithinTolerance(vector8, vector7));
}

TEST(TestUtilsTest, angle_check_if_equal_within_tolerance)
{
Angle angle1 = Angle::fromDegrees(5);
Angle angle2 = Angle::fromDegrees(5.5);
Angle angle3 = Angle::fromDegrees(189);
Angle angle4 = Angle::fromDegrees(190);
Angle angle5 = Angle::fromDegrees(-10);

EXPECT_TRUE(
        ::equalWithinTolerance(angle1, angle2, Angle::fromDegrees(0.5)));
EXPECT_FALSE(
        ::equalWithinTolerance(angle1, angle2, Angle::fromDegrees(0.49)));
EXPECT_TRUE(::equalWithinTolerance(angle3, angle4, Angle::fromDegrees(1)));
EXPECT_FALSE(
        ::equalWithinTolerance(angle3, angle4, Angle::fromDegrees(0.99)));
EXPECT_TRUE(::equalWithinTolerance(angle1, angle5, Angle::fromDegrees(15)));
EXPECT_TRUE(
        ::equalWithinTolerance(angle4, angle5, Angle::fromDegrees(180)));
}

TEST(TestUtilsTest, robot_state_check_if_equal_within_tolerance)
{
RobotState state1(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(4),
                  AngularVelocity::fromDegrees(67.4));
RobotState state2(Point(1.01, 2.59), Vector(0.005, -2.05), Angle::fromDegrees(5),
                  AngularVelocity::fromDegrees(67.6));
RobotState state3(Point(1.11, 2.59), Vector(0.0, -2.06), Angle::fromDegrees(4),
                  AngularVelocity::fromDegrees(67.4));
RobotState state4(Point(1.01, 2.58), Vector(0.4, -2.58), Angle::fromDegrees(4),
                  AngularVelocity::fromDegrees(67.4));
RobotState state5(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(6.4),
                  AngularVelocity::fromDegrees(67.4));
RobotState state6(Point(1.01, 2.58), Vector(0.0, -2.06), Angle::fromDegrees(4),
                  AngularVelocity::fromDegrees(70.02));

EXPECT_TRUE(
        equalWithinTolerance(state1, state2, 0.02, Angle::fromDegrees(1)));
EXPECT_FALSE(
        equalWithinTolerance(state1, state3, 0.1, Angle::fromDegrees(1)));
EXPECT_FALSE(
        equalWithinTolerance(state1, state4, 0.5, Angle::fromDegrees(1)));
EXPECT_FALSE(
        equalWithinTolerance(state1, state5, 1e-3, Angle::fromDegrees(2)));
EXPECT_FALSE(
        equalWithinTolerance(state1, state6, 1e-3, Angle::fromDegrees(2.5)));
}

TEST(TestUtilsTest, ball_state_check_if_equal_within_tolerance)
{
BallState state1(Point(1.01, 2.58), Vector(0.0, -2.06));
BallState state2(Point(1.01, 2.59), Vector(0.005, -2.05));
BallState state3(Point(1.11, 2.59), Vector(0.0, -2.06));
BallState state4(Point(1.01, 2.58), Vector(0.4, -2.58));

EXPECT_TRUE(equalWithinTolerance(state1, state2, 0.02));
EXPECT_FALSE(equalWithinTolerance(state1, state3, 0.1));
EXPECT_FALSE(equalWithinTolerance(state1, state4, 0.5));
}