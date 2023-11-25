#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

#include <gtest/gtest.h>

#include <iostream>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

class RobotNavigationObstacleFactoryTest : public testing::Test
{
   public:
    RobotNavigationObstacleFactoryTest()
        : current_time(Timestamp::fromSeconds(123)),
          robot_navigation_obstacle_factory(config)
    {
        config.set_robot_obstacle_inflation_factor(1.3);
        robot_navigation_obstacle_factory = RobotNavigationObstacleFactory(config);
    }

    Timestamp current_time;
    TbotsProto::RobotNavigationObstacleConfig config;
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
};

class RobotNavigationObstacleFactoryMotionConstraintTest : public testing::Test
{
   public:
    RobotNavigationObstacleFactoryMotionConstraintTest()
        : current_time(
              Timestamp::fromSeconds(123)),  // arbitrary point in time for testing
          field(Field::createSSLDivisionBField()),
          ball(Point(1, 2), Vector(-0.3, 0), current_time),
          friendly_team(Duration::fromMilliseconds(1000)),
          enemy_team(Duration::fromMilliseconds(1000)),
          world(field, ball, friendly_team, enemy_team),
          robot_navigation_obstacle_factory(robot_navigation_obstacle_config)
    {
        robot_navigation_obstacle_config.set_robot_obstacle_inflation_factor(1.3);
        robot_navigation_obstacle_factory =
            RobotNavigationObstacleFactory(robot_navigation_obstacle_config);
    }

    void SetUp() override
    {
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
    RobotNavigationObstacleFactory robot_navigation_obstacle_factory;
    TbotsProto::RobotNavigationObstacleConfig robot_navigation_obstacle_config;
};

TEST_F(RobotNavigationObstacleFactoryTest, create_rectangle_obstacle)
{
    Rectangle rectangle(Point(1, 3), Point(5, 8));
    Rectangle expected({0.883, 2.883}, {5.117, 8.117});
    ObstaclePtr obstacle = robot_navigation_obstacle_factory.createFromShape(rectangle);
    try
    {
        auto rectangle_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacle);
        EXPECT_EQ(expected, rectangle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Rectangle Obstacle was not created for a rectangle";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, create_ball_obstacle)
{
    Point origin(2.5, 4);
    Circle expected(origin, 0.1385);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromBallPosition(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a ball";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, create_robot_obstacle)
{
    Point origin(2.5, 4);
    Circle expected(origin, 0.207);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromRobotPosition(origin);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, stationary_robot_obstacle)
{
    Point origin(2.3, 3);
    Vector velocity(0.0, 0.0);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromRobotPosition(robot.position());

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "GeomObstacle<Circle>Ptr was not created for a stationary robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, slow_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(0.0007, 0.004);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromRobotPosition(robot.position());

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a slow moving robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, fast_moving_robot_obstacle)
{
    Point origin(-2.1, 5);
    Vector velocity(1.27, 0.34);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromRobotPosition(robot.position());

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a fast moving robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, another_fast_moving_robot_obstacle)
{
    Point origin(1.2, -0.2);
    Vector velocity(-0.27, 1.34);
    Angle orientation(Angle::fromRadians(-1.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(1.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(4, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromRobotPosition(robot.position());

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        EXPECT_EQ(expected, circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a fast moving robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, centre_circle)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::CENTER_CIRCLE, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Circle expected({0, 0}, 0.617);
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Circle Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, half_metre_around_ball)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createDynamicObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Circle expected({1, 2}, 0.617);
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, circle_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Circle Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, inflated_enemy_defense_area)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Rectangle expected(Point(3.083, -1.417), Point(4.8, 1.417));
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, friendly_defense_area)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Rectangle expected(Point(-4.8, -1.117), Point(-3.383, 1.117));
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, enemy_defense_area)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Rectangle expected(Point(3.383, -1.117), Point(4.8, 1.117));
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, friendly_half)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_HALF, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Rectangle expected(Point(-4.8, -3.3), Point(0.117, 3.3));
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, enemy_half)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createStaticObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::ENEMY_HALF, world.field());
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Rectangle expected(Point(-0.117, -3.3), Point(4.8, 3.3));
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Rectangle>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, ball_placement_rectangle)
{
    Ball new_ball               = Ball(Point(0, 0), Vector(0, 0), current_time);
    Point placement_point       = Point(1, 0);
    GameState ball_placement_gs = world.gameState();
    ball_placement_gs.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    ball_placement_gs.setBallPlacementPoint(placement_point);
    ball_placement_gs.updateBall(new_ball);
    world.updateBall(new_ball);
    world.updateGameState(ball_placement_gs);
    auto obstacles =
        robot_navigation_obstacle_factory.createDynamicObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected(
            {{-0.617, 0.617}, {1.617, 0.617}, {1.617, -0.617}, {-0.617, -0.617}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
        // check for 90 degrees between
        // +---------+ <-+
        // |<-width->|   | depth
        // +---------+ <-+
        std::vector<Point> points = polygon_obstacle.getGeom().getPoints();
        Vector width              = points[0] - points[1];
        Vector depth              = points[2] - points[1];
        EXPECT_NEAR(depth.dot(width), 0, 1e-10);
        // check that box covers ball placement zone that cannot be entered
        EXPECT_GE(width.length(), 1 + 2 * ROBOT_MAX_RADIUS_METERS);
    }

    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, ball_placement_rotated)
{
    Ball new_ball               = Ball(Point(0, 0), Vector(0, 0), current_time);
    Point placement_point       = Point(1, 1);
    GameState ball_placement_gs = world.gameState();
    ball_placement_gs.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    ball_placement_gs.setBallPlacementPoint(placement_point);
    ball_placement_gs.updateBall(new_ball);
    world.updateBall(new_ball);
    world.updateGameState(ball_placement_gs);
    auto obstacles =
        robot_navigation_obstacle_factory.createDynamicObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE, world);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Polygon expected({{-0.873, 0}, {1, 1.873}, {1.873, 1}, {0, -0.873}});
        auto polygon_obstacle = dynamic_cast<GeomObstacle<Polygon>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, polygon_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
        // check for 90 degrees between
        // +---------+ <-+
        // |<-width->|   | depth
        // +---------+ <-+
        std::vector<Point> points = polygon_obstacle.getGeom().getPoints();
        Vector width              = points[0] - points[1];
        Vector depth              = points[2] - points[1];
        // check that box covers ball placement zone that cannot be entered
        EXPECT_NEAR(depth.dot(width), 0, 1e-10);
        EXPECT_GE(width.length(), 1 + 2 * ROBOT_MAX_RADIUS_METERS);
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Polygon Obstacle was not created";
    }
}
