#include "software/ai/navigator/obstacle/obstacle.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "shared/constants.h"
#include "software/new_geom/point.h"

// obstacle with robot radius factor = 1, velocity projection factor = 1
// centred at (0,0) and with (0,0) velocity
TEST(NavigatorObstacleTest, default_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot    = Robot(3, Point(0.0, 0.0), Vector(0.0, 0.0), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);
    auto p_pointer = Obstacle::createRobotObstacleWithScalingParams(robot, 1.0, 1.0)
                         .getBoundaryPolygon();

    EXPECT_TRUE(p_pointer);

    if (p_pointer)
    {
        Polygon pg = *p_pointer;
        // visually verified
        Point pt0 = Point(0, 0.208);
        Point pt1 = Point(-0.18, 0.104);
        Point pt2 = Point(-0.18, -0.104);
        Point pt3 = Point(0, -0.208);
        Point pt4 = Point(0.18, -0.104);
        Point pt5 = Point(0.18, 0.104);
        EXPECT_DOUBLE_EQ(pg.getPoints().size(), 6);
        EXPECT_NEAR(pg.getPoints()[0].x(), pt0.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[0].y(), pt0.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].x(), pt1.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].y(), pt1.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].x(), pt2.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].y(), pt2.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].x(), pt3.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].y(), pt3.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].x(), pt4.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].y(), pt4.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].x(), pt5.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].y(), pt5.y(), TEST_EPSILON);
    }
}

// obstacle with robot radius factor = 1, velocity projection factor = 1
// centred at (1,2) and with (3,2) velocity
TEST(NavigatorObstacleTest, shifted_scaling_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot            = Robot(3, Point(1, 2), Vector(3, 2), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);
    auto p_pointer = Obstacle::createRobotObstacleWithScalingParams(robot, 1.0, 1.0)
                         .getBoundaryPolygon();

    EXPECT_TRUE(p_pointer);

    if (p_pointer)
    {
        Polygon pg = *p_pointer;
        // visually verified
        Point pt0 = Point(0.885, 2.173);
        Point pt1 = Point(.793, 1.987);
        Point pt2 = Point(.908, 1.814);
        Point pt3 = Point(1.115, 1.827);
        Point pt4 = Point(4.265, 3.927);
        Point pt5 = Point(4.034, 4.273);
        EXPECT_DOUBLE_EQ(pg.getPoints().size(), 6);
        EXPECT_NEAR(pg.getPoints()[0].x(), pt0.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[0].y(), pt0.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].x(), pt1.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].y(), pt1.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].x(), pt2.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].y(), pt2.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].x(), pt3.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].y(), pt3.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].x(), pt4.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].y(), pt4.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].x(), pt5.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].y(), pt5.y(), TEST_EPSILON);
    }
}

// obstacle with robot radius factor = 1.2, velocity projection factor = 1.4
// centred at (-1,-2) and with (-3,2) velocity
TEST(NavigatorObstacleTest, shifted_scaled_up_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot    = Robot(3, Point(-1, -2), Vector(-3, 2), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);
    auto p_pointer = Obstacle::createRobotObstacleWithScalingParams(robot, 1.2, 1.4)
                         .getBoundaryPolygon();

    EXPECT_TRUE(p_pointer);

    if (p_pointer)
    {
        Polygon pg = *p_pointer;
        // visually verified
        Point pt0 = Point(-1.138, -2.208);
        Point pt1 = Point(-.889, -2.224);
        Point pt2 = Point(-0.751, -2.016);
        Point pt3 = Point(-0.862, -1.792);
        Point pt4 = Point(-5.241, 1.127);
        Point pt5 = Point(-5.518, 0.712);
        EXPECT_DOUBLE_EQ(pg.getPoints().size(), 6);
        EXPECT_NEAR(pg.getPoints()[0].x(), pt0.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[0].y(), pt0.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].x(), pt1.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].y(), pt1.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].x(), pt2.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].y(), pt2.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].x(), pt3.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].y(), pt3.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].x(), pt4.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].y(), pt4.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].x(), pt5.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].y(), pt5.y(), TEST_EPSILON);
    }
}

// obstacle with robot additional radius buffer = 0, additional velocity projection buffer
// = 0 centred at (1,2) and with (3,2) velocity
TEST(NavigatorObstacleTest, shifted_no_buffer_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot            = Robot(3, Point(1, 2), Vector(3, 2), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);
    auto p_pointer = Obstacle::createRobotObstacleWithBufferParams(robot, true, 0.0, 0.0)
                         .getBoundaryPolygon();

    EXPECT_TRUE(p_pointer);

    if (p_pointer)
    {
        Polygon pg = *p_pointer;
        // visually verified
        Point pt0 = Point(0.885, 2.173);
        Point pt1 = Point(.793, 1.987);
        Point pt2 = Point(.908, 1.814);
        Point pt3 = Point(1.115, 1.827);
        Point pt4 = Point(4.115, 3.827);
        Point pt5 = Point(3.885, 4.173);
        EXPECT_DOUBLE_EQ(pg.getPoints().size(), 6);
        EXPECT_NEAR(pg.getPoints()[0].x(), pt0.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[0].y(), pt0.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].x(), pt1.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].y(), pt1.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].x(), pt2.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].y(), pt2.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].x(), pt3.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].y(), pt3.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].x(), pt4.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].y(), pt4.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].x(), pt5.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].y(), pt5.y(), TEST_EPSILON);
    }
}

// obstacle with robot additional radius buffer = 0.4, additional velocity projection
// buffer = 1.8 centred at (-1,-2) and with (-3,2) velocity
TEST(NavigatorObstacleTest, shifted_with_buffer_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot    = Robot(3, Point(-1, -2), Vector(-3, 2), Angle::fromRadians(2.2),
                        AngularVelocity::fromRadians(-0.6), current_time);
    auto p_pointer = Obstacle::createRobotObstacleWithBufferParams(robot, true, 0.4, 1.8)
                         .getBoundaryPolygon();

    EXPECT_TRUE(p_pointer);

    if (p_pointer)
    {
        Polygon pg = *p_pointer;
        // visually verified
        Point pt0 = Point(-1.628, -2.941);
        Point pt1 = Point(-0.498, -3.014);
        Point pt2 = Point(0.129, -2.073);
        Point pt3 = Point(-0.372, -1.058);
        Point pt4 = Point(-4.87, 1.94);
        Point pt5 = Point(-6.125, 0.057);
        EXPECT_DOUBLE_EQ(pg.getPoints().size(), 6);
        EXPECT_NEAR(pg.getPoints()[0].x(), pt0.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[0].y(), pt0.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].x(), pt1.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[1].y(), pt1.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].x(), pt2.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[2].y(), pt2.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].x(), pt3.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[3].y(), pt3.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].x(), pt4.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[4].y(), pt4.y(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].x(), pt5.x(), TEST_EPSILON);
        EXPECT_NEAR(pg.getPoints()[5].y(), pt5.y(), TEST_EPSILON);
    }
}

TEST(NavigatorObstacleTest, create_from_rectangle)
{
    Obstacle obstacle(Rectangle({-1, 1}, {2, -3}));

    Polygon expected = Polygon({
        {-1, -3},
        {-1, 1},
        {2, 1},
        {2, -3},
    });
    EXPECT_TRUE((bool)obstacle.getBoundaryPolygon());
    EXPECT_EQ(expected.getPoints(), (*obstacle.getBoundaryPolygon()).getPoints());
}

TEST(NavigatorObstacleTest, create_from_circle)
{
    Obstacle obstacle = Obstacle::createCircleObstacle({2, 2}, 1, 1);

    EXPECT_FALSE(obstacle.isPolygon());
    EXPECT_TRUE(obstacle.getBoundaryCircle());

    EXPECT_EQ((*obstacle.getBoundaryCircle()).getRadius(), (1 + ROBOT_MAX_RADIUS_METERS));
    EXPECT_EQ((*obstacle.getBoundaryCircle()).getOrigin(), Point(2, 2));
}
