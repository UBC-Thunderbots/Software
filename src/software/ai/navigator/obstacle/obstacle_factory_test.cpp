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

class ObstacleFactoryMotionConstraintTest : public testing::Test
{
   public:
    ObstacleFactoryMotionConstraintTest()
        : current_time(Timestamp::fromSeconds(123)),
          obstacle_factory(
              Util::DynamicParameters->getAIConfig()->getObstacleFactoryConfig()),
          field(),
          ball(Point(1, 2), Vector(-0.3, 0), current_time),
          friendly_team(Duration::fromMilliseconds(1000)),
          enemy_team(Duration::fromMilliseconds(1000)),
          world(field, ball, friendly_team, enemy_team)
    {
    }

    void SetUp() override
    {
        // An arbitrary fixed point in time
        // We use this fixed point in time to make the tests deterministic.
        field = ::Test::TestUtil::createSSLDivBField();

        Robot friendly_robot_0 = Robot(0, Point(0, 1), Vector(-1, -2), Angle::half(),
                                       AngularVelocity::threeQuarter(), current_time);

        Robot friendly_robot_1 = Robot(1, Point(3, -1), Vector(), Angle::zero(),
                                       AngularVelocity::zero(), current_time);

        friendly_team.updateRobots({friendly_robot_0, friendly_robot_1});
        friendly_team.assignGoalie(1);

        Robot enemy_robot_0 = Robot(0, Point(0.5, -2.5), Vector(), Angle::fromRadians(1),
                                    AngularVelocity::fromRadians(2), current_time);

        Robot enemy_robot_1 = Robot(1, Point(), Vector(-0.5, 4), Angle::quarter(),
                                    AngularVelocity::half(), current_time);

        enemy_team.updateRobots({enemy_robot_0, enemy_robot_1});
        enemy_team.assignGoalie(0);

        // Construct the world with arguments
        world = World(field, ball, friendly_team, enemy_team);
    }

    Timestamp current_time;
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
    World world;
    ObstacleFactory obstacle_factory;
};

TEST_F(ObstacleFactoryTest, create_rectangle_obstacle)
{
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Polygon expected(Rectangle(Point(.883, 2.883), Point(5.117, 8.117)));
    ObstaclePtr obstacle = obstacle_factory.createInflatedByRobotRadius(rectangle);

    try
    {
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacle);
        EXPECT_EQ(expected, polygon_obstacle.getGeom());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created for a rectangle";
    }
}

TEST_F(ObstacleFactoryTest, create_ball_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.1385);
    ObstaclePtr obstacle = obstacle_factory.createAroundBallPosition(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a ball";
    }
}

TEST_F(ObstacleFactoryTest, create_robot_obstacle)
{
    Point origin(2.5, 4);
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Circle expected(origin, 0.207);
    ObstaclePtr obstacle = obstacle_factory.createAroundRobotPosition(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a robot";
    }
}

TEST_F(ObstacleFactoryTest, stationary_robot_obstacle)
{
    Point origin(2.3, 3);
    Vector velocity(0.0, 0.0);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(
            expected, circle_obstacle.getGeom(), METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a stationary robot";
    }
}

TEST_F(ObstacleFactoryTest, slow_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(0.0007, 0.004);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a slow moving robot";
    }
}

TEST_F(ObstacleFactoryTest, fast_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(1.27, 0.34);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Polygon expected({Point(-2.161, 5.231), Point(-2.331, 5.062), Point(-2.269, 4.831),
                      Point(-2.038, 4.769), Point(-1.671, 4.867), Point(-1.795, 5.329)});
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacle);
        EXPECT_TRUE(::Test::TestUtil::equalWithinTolerance(
            expected, polygon_obstacle.getGeom(), METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created for a fast moving robot";
    }
}

TEST_F(ObstacleFactoryTest, another_fast_moving_robot_obstacle)
{
    Point origin(1.2, -0.2);
    Vector velocity(-0.27, 1.34);
    Angle orientation(Angle::fromRadians(-1.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(1.6));
    Polygon expected({Point(0.966, -0.247), Point(1.124, -0.427), Point(1.358, -0.379),
                      Point(1.434, -0.153), Point(1.357, 0.230), Point(0.889, 0.135)});
    Robot robot = Robot(4, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle = obstacle_factory.createVelocityObstacleFromRobot(robot);

    try
    {
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacle);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created for a fast moving robot";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, enemy_robots_collision)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::ENEMY_ROBOTS_COLLISION, world);
    EXPECT_EQ(2, obstacles.size());
    try
    {
        Circle expected({0.5, -2.5}, 0.207);
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Circle Obstacle was not created for stationary enemy robot";
    }

    try
    {
        Polygon expected({{-0.237, -0.030},
                          {-0.092, -0.220},
                          {0.144, -0.191},
                          {0.237, 0.0296},
                          {0.123, 0.946},
                          {-0.352, 0.886}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[1]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created for a fast moving robot";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, centre_circle)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::CENTER_CIRCLE, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Circle expected({0, 0}, 0.617);
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Circle Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, half_metre_around_ball)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::HALF_METER_AROUND_BALL, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Circle expected({1, 2}, 0.617);
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Circle Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, inflated_enemy_defense_area)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected({{3.083, -1.417}, {3.083, 1.417}, {4.8, 1.417}, {4.8, -1.417}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, friendly_defense_area)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::FRIENDLY_DEFENSE_AREA, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected(
            {{-4.8, -1.117}, {-4.8, 1.117}, {-3.383, 1.117}, {-3.383, -1.117}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, enemy_defense_area)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::ENEMY_DEFENSE_AREA, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected({{3.383, -1.117}, {3.383, 1.117}, {4.8, 1.117}, {4.8, -1.117}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, friendly_half)
{
    auto obstacles = obstacle_factory.createFromMotionConstraint(
        MotionConstraint::FRIENDLY_HALF, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected({{-4.8, -3.3}, {-4.8, 3.3}, {0.117, 3.3}, {0.117, -3.3}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(ObstacleFactoryMotionConstraintTest, enemy_half)
{
    auto obstacles =
        obstacle_factory.createFromMotionConstraint(MotionConstraint::ENEMY_HALF, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected({{-0.117, -3.3}, {-0.117, 3.3}, {4.8, 3.3}, {4.8, -3.3}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(
            ::Test::TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom()));
    }
    catch (std::bad_cast)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}
