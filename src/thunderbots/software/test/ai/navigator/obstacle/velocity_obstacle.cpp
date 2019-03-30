#include "ai/navigator/obstacle/velocity_obstacle.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "../shared/constants.h"
#include "geom/point.h"

// velocity obstacle with robot radius factor = 1, velocity projection factor = 1
// centred at (0,0) and with (0,0) velocity
TEST(NavigatorVelocityObstacleTest, default_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(0.0, 0.0), Vector(0.0, 0.0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg  = VelocityObstacle::getBoundaryPolygon(robot, 1.0, 1.0);
    Point pt0   = Point(0, ROBOT_MAX_RADIUS_METERS);
    Point pt1   = Point(0, -ROBOT_MAX_RADIUS_METERS);
    // visually verified
    Point pt2 = Point(0.078, 0.045);
    Point pt3 = Point(0.078, -0.045);
    Point pt4 = Point(-0.078, 0.045);
    Point pt5 = Point(-0.078, -0.045);
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

// velocity obstacle with robot radius factor = 1, velocity projection factor = 1
// centred at (1,2) and with (3,2) velocity
TEST(NavigatorVelocityObstacleTest, shifted_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot            = Robot(3, Point(1, 2), Vector(3, 2), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg             = VelocityObstacle::getBoundaryPolygon(robot, 1.0, 1.0);
    // visually verified
    Point pt0 = Point(0.950, 2.075);
    Point pt1 = Point(1.05, 1.925);
    Point pt2 = Point(3.95, 4.075);
    Point pt3 = Point(4.05, 3.925);
    Point pt4 = Point(0.910, 1.994);
    Point pt5 = Point(0.96, 1.919);
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

// velocity obstacle with robot radius factor = 1.2, velocity projection factor = 1.4
// centred at (-1,-2) and with (-3,2) velocity
TEST(NavigatorVelocityObstacleTest, shifted_scaled_up_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot            = Robot(3, Point(-1, -2), Vector(-3, 2), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg             = VelocityObstacle::getBoundaryPolygon(robot, 1.2, 1.4);
    Point pt0              = Point(-1.06, -2.09);
    Point pt1              = Point(-0.94, -1.91);
    Point pt2              = Point(-5.26, 0.71);
    Point pt3              = Point(-5.14, 0.89);
    Point pt4              = Point(-0.952, -2.097);
    Point pt5              = Point(-0.892, -2.006);
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

// velocity obstacle with robot radius factor = 0.3, velocity projection factor = 0.4
// centred at (-1,-2) and with (-3,2) velocity
TEST(NavigatorVelocityObstacleTest, shifted_scaled_down_velocity_obstacle_polygon)
{
    double TEST_EPSILON    = 1e-3;
    Timestamp current_time = Timestamp::fromSeconds(123);
    Robot robot            = Robot(3, Point(-1, -2), Vector(-3, 2), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg             = VelocityObstacle::getBoundaryPolygon(robot, .3, .4);
    Point pt0              = Point(-1.014, -2.022);
    Point pt1              = Point(-0.985, -1.977);
    Point pt2              = Point(-2.215, -1.222);
    Point pt3              = Point(-2.185, -1.178);
    Point pt4              = Point(-0.988, -2.024);
    Point pt5              = Point(-0.973, -2.002);
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

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
