#include "ai/navigator/obstacle/placeholder_obstacle.h"

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>

#include "../shared/constants.h"
#include "geom/point.h"

// placeholder obstacle with robot radius factor = 1
TEST(NavigatorPlaceholderObstacleTest, default_placeholder_obstacle_polygon)
{
    double ROOT_THREE_BY_TWO = 0.86602540378;
    Timestamp current_time   = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg  = PlaceholderObstacle::getBoundaryPolygon(robot, 1.0, 3.0);
    Point pt0   = Point(ROBOT_MAX_RADIUS_METERS, 0);
    Point pt1   = Point(-ROBOT_MAX_RADIUS_METERS, 0);
    Point pt2 =
        Point(ROBOT_MAX_RADIUS_METERS / 2.0, ROBOT_MAX_RADIUS_METERS * ROOT_THREE_BY_TWO);
    Point pt3 = Point(-ROBOT_MAX_RADIUS_METERS / 2.0,
                      ROBOT_MAX_RADIUS_METERS * ROOT_THREE_BY_TWO);
    Point pt4 = Point(ROBOT_MAX_RADIUS_METERS / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * ROOT_THREE_BY_TWO);
    Point pt5 = Point(-ROBOT_MAX_RADIUS_METERS / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * ROOT_THREE_BY_TWO);
    EXPECT_EQ(pg.getPoints().size(), 6);
    EXPECT_EQ(pg.getPoints()[0], pt0);
    EXPECT_EQ(pg.getPoints()[1], pt1);
    EXPECT_EQ(pg.getPoints()[2], pt2);
    EXPECT_EQ(pg.getPoints()[3], pt3);
    EXPECT_EQ(pg.getPoints()[4], pt4);
    EXPECT_EQ(pg.getPoints()[5], pt5);
}

// placeholder obstacle with robot radius factor > 1
TEST(NavigatorPlaceholderObstacleTest, expanded_placeholder_obstacle_polygon)
{
    double EXPANSION         = 1.4;
    double ROOT_THREE_BY_TWO = 0.86602540378;
    Timestamp current_time   = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg  = PlaceholderObstacle::getBoundaryPolygon(robot, EXPANSION, 3.0);
    Point pt0   = Point(ROBOT_MAX_RADIUS_METERS * EXPANSION, 0);
    Point pt1   = Point(-ROBOT_MAX_RADIUS_METERS * EXPANSION, 0);
    Point pt2   = Point(ROBOT_MAX_RADIUS_METERS * EXPANSION / 2.0,
                      ROBOT_MAX_RADIUS_METERS * EXPANSION * ROOT_THREE_BY_TWO);
    Point pt3   = Point(-ROBOT_MAX_RADIUS_METERS * EXPANSION / 2.0,
                      ROBOT_MAX_RADIUS_METERS * EXPANSION * ROOT_THREE_BY_TWO);
    Point pt4   = Point(ROBOT_MAX_RADIUS_METERS * EXPANSION / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * EXPANSION * ROOT_THREE_BY_TWO);
    Point pt5   = Point(-ROBOT_MAX_RADIUS_METERS * EXPANSION / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * EXPANSION * ROOT_THREE_BY_TWO);
    EXPECT_EQ(pg.getPoints().size(), 6);
    EXPECT_EQ(pg.getPoints()[0], pt0);
    EXPECT_EQ(pg.getPoints()[1], pt1);
    EXPECT_EQ(pg.getPoints()[2], pt2);
    EXPECT_EQ(pg.getPoints()[3], pt3);
    EXPECT_EQ(pg.getPoints()[4], pt4);
    EXPECT_EQ(pg.getPoints()[5], pt5);
}

// placeholder obstacle with robot radius factor < 1
TEST(NavigatorPlaceholderObstacleTest, contracted_placeholder_obstacle_polygon)
{
    double CONTRACTION       = .7;
    double ROOT_THREE_BY_TWO = 0.86602540378;
    Timestamp current_time   = Timestamp::fromSeconds(123);
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6), current_time);
    Polygon pg  = PlaceholderObstacle::getBoundaryPolygon(robot, CONTRACTION, 3.0);
    Point pt0   = Point(ROBOT_MAX_RADIUS_METERS * CONTRACTION, 0);
    Point pt1   = Point(-ROBOT_MAX_RADIUS_METERS * CONTRACTION, 0);
    Point pt2   = Point(ROBOT_MAX_RADIUS_METERS * CONTRACTION / 2.0,
                      ROBOT_MAX_RADIUS_METERS * CONTRACTION * ROOT_THREE_BY_TWO);
    Point pt3   = Point(-ROBOT_MAX_RADIUS_METERS * CONTRACTION / 2.0,
                      ROBOT_MAX_RADIUS_METERS * CONTRACTION * ROOT_THREE_BY_TWO);
    Point pt4   = Point(ROBOT_MAX_RADIUS_METERS * CONTRACTION / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * CONTRACTION * ROOT_THREE_BY_TWO);
    Point pt5   = Point(-ROBOT_MAX_RADIUS_METERS * CONTRACTION / 2.0,
                      -ROBOT_MAX_RADIUS_METERS * CONTRACTION * ROOT_THREE_BY_TWO);
    EXPECT_EQ(pg.getPoints().size(), 6);
    EXPECT_EQ(pg.getPoints()[0], pt0);
    EXPECT_EQ(pg.getPoints()[1], pt1);
    EXPECT_EQ(pg.getPoints()[2], pt2);
    EXPECT_EQ(pg.getPoints()[3], pt3);
    EXPECT_EQ(pg.getPoints()[4], pt4);
    EXPECT_EQ(pg.getPoints()[5], pt5);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
