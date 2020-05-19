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
          obstacle_factory(
              Util::DynamicParameters->getAIConfig()->getObstacleFactoryConfig())
    {
    }

    Timestamp current_time;
    ObstacleFactory obstacle_factory;
};

TEST_F(ObstacleFactoryTest, create_rectangle_obstacle)
{
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Polygon expected(Rectangle(Point(.883, 2.883), Point(5.117, 8.117)));
    ObstaclePtr obstacle = obstacle_factory.createObstacleFromRectangle(rectangle);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_EQ(expected, polygon_obstacle.getPolygon());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstaclePtr was not created for a rectangle";
    }
}

TEST_F(ObstacleFactoryTest, create_ball_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.1985);
    ObstaclePtr obstacle = obstacle_factory.createBallObstacle(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstaclePtr was not created for a ball";
    }
}

TEST_F(ObstacleFactoryTest, create_robot_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.207);
    ObstaclePtr obstacle = obstacle_factory.createRobotObstacle(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstaclePtr was not created for a robot";
    }
}

TEST_F(ObstacleFactoryTest, stationary_robot_obstacle)
{
    Point origin(2.3, 3);
    Vector velocity(0.0, 0.0);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.042);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(
            expected, circle_obstacle.getCircle(), METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstaclePtr was not created for a stationary robot";
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
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<CircleObstacle&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getCircle());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "CircleObstaclePtr was not created for a slow moving robot";
    }
}

TEST_F(ObstacleFactoryTest, fast_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(0.27, 0.34);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Polygon expected({Point(-2.133, 5.026), Point(-2.139, 4.985), Point(-2.106, 4.959),
                      Point(-2.067, 4.974), Point(-1.694, 5.444), Point(-1.759, 5.496)});
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(
            expected, polygon_obstacle.getPolygon(), METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstaclePtr was not created for a fast moving robot";
    }
}

TEST_F(ObstacleFactoryTest, another_fast_moving_robot_obstacle)
{
    Point origin(1.2, -0.2);
    Vector velocity(-0.27, 1.34);
    Angle orientation(Angle::fromRadians(-1.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(1.6));
    Polygon expected({Point(1.159, -0.208), Point(1.187, -0.239), Point(1.227, -0.231),
                      Point(1.241, -0.192), Point(0.883, 1.586), Point(0.801, 1.569)});
    Robot robot = Robot(4, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<PolygonObstacle&>(*obstacle);
        EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(
            expected, polygon_obstacle.getPolygon()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "PolygonObstaclePtr was not created for a fast moving robot";
    }
}
