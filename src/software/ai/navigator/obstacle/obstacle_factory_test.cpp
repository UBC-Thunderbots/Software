#include "software/ai/navigator/obstacle/obstacle_factory.h"

#include <gtest/gtest.h>

#include <iostream>

#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/rectangle.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"


class ObstacleFactoryTest : public testing::Test
{
   public:
    ObstacleFactoryTest()
        : current_time(Timestamp::fromSeconds(123)),
          obstacle_factory(std::make_shared<ObstacleFactoryConfig>())
    {
    }

    ObstacleFactory obstacle_factory;
    Timestamp current_time;
};

TEST_F(ObstacleFactoryTest, create_rectangle_obstacle)
{
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Polygon expected(Rectangle(Point(.883, 2.883), Point(5.117, 8.117)));
    Obstacle obstacle = obstacle_factory.createObstacleFromRectangle(rectangle);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_EQ(expected, polygon_obstacle.getPolygon());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstacle was not created for a rectangle";
    }
}

TEST_F(ObstacleFactoryTest, create_ball_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.1985);
    Obstacle obstacle = obstacle_factory.createBallObstacle(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstacle was not created for a ball";
    }
}

TEST_F(ObstacleFactoryTest, create_robot_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.207);
    Obstacle obstacle = obstacle_factory.createRobotObstacle(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstacle was not created for a robot";
    }
}

TEST_F(ObstacleFactoryTest, stationary_robot_obstacle)
{
    Point origin(2.3, 3);
    Vector velocity(0.0, 0.0);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.041569219381653054);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    Obstacle obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstacle was not created for a stationary robot";
    }
}

TEST_F(ObstacleFactoryTest, slow_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(0.0007, 0.004);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.041569219381653054);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    Obstacle obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstacle was not created for a slow moving robot";
    }
}

TEST_F(ObstacleFactoryTest, fast_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(0.27, 0.34);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Polygon expected({Point(-2.1325533066198088, 5.0258511552569072),
                      Point(-2.1386644104795618, 4.984733587118515),
                      Point(-2.1061111038597526, 4.9588824318616078),
                      Point(-2.0674466933801914, 4.9741488447430928),
                      Point(-1.694058936210534, 5.4443408352530316),
                      Point(-1.7591655494501515, 5.4960431457668459)});
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    Obstacle obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_EQ(expected, polygon_obstacle.getPolygon());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstacle was not created for a fast moving robot";
    }
}

TEST_F(ObstacleFactoryTest, another_fast_moving_robot_obstacle)
{
    Point origin(1.2, -0.2);
    Vector velocity(-0.27, 1.34);
    Angle orientation(Angle::fromRadians(-1.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(1.6));
    Polygon expected({Point(1.1592497651168834, -0.20821086822271753),
                      Point(1.1867357030264414, -0.23939617273032049),
                      Point(1.227485937909558, -0.23118530450760297),
                      Point(1.2407502348831165, -0.19178913177728252),
                      Point(0.88263941441511673, 1.5855016068416794),
                      Point(0.80113894464888358, 1.5690798703962443)});
    Robot robot = Robot(4, origin, velocity, orientation, angular_velocity, current_time);
    Obstacle obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_EQ(expected, polygon_obstacle.getPolygon());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstacle was not created for a fast moving robot";
    }
}
