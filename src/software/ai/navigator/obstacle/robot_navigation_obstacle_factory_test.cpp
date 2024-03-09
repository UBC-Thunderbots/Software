#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

#include <gtest/gtest.h>

#include <iostream>

#include "software/ai/navigator/obstacle/const_velocity_obstacle.hpp"
#include "software/ai/navigator/obstacle/geom_obstacle.hpp"
#include "software/ai/navigator/obstacle/trajectory_obstacle.hpp"
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
        config.set_dynamic_enemy_robot_obstacle_horizon_sec(1.0);
        config.set_dynamic_enemy_robot_obstacle_min_speed_mps(0.5);
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
          world_ptr(std::make_shared<World>(field, ball, friendly_team, enemy_team)),
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

        // Construct the world_ptr with arguments
        world_ptr =
            std::make_shared<World>(World(field, ball, friendly_team, enemy_team));
    }

    Timestamp current_time;
    Field field;
    Ball ball;
    Team friendly_team;
    Team enemy_team;
    std::shared_ptr<World> world_ptr;
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

TEST_F(RobotNavigationObstacleFactoryTest, static_robot_obstacle_1)
{
    Point origin(2.5, 4);
    Circle expected(origin, 0.207);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createStaticObstacleFromRobotPosition(origin);

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

TEST_F(RobotNavigationObstacleFactoryTest, static_robot_obstacle_2)
{
    Point origin(2.3, 3);
    Vector velocity(0.0, 0.0);
    Angle orientation(Angle::fromRadians(2.2));
    AngularVelocity angular_velocity(AngularVelocity::fromRadians(-0.6));
    Circle expected(origin, 0.207);
    Robot robot = Robot(3, origin, velocity, orientation, angular_velocity, current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createStaticObstacleFromRobotPosition(
            robot.position());

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

TEST_F(RobotNavigationObstacleFactoryTest, stadium_enemy_robot_obstacle)
{
    Point origin(1.0, 1.0);
    Vector velocity(1.0, 0.0);
    Robot robot =
        Robot(4, origin, velocity, Angle::zero(), AngularVelocity::zero(), current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createStadiumEnemyRobotObstacle(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Stadium>&>(*obstacle);
        TestUtil::equalWithinTolerance(
            Stadium(origin,
                    origin + velocity * config.dynamic_enemy_robot_obstacle_horizon_sec(),
                    ROBOT_MAX_RADIUS_METERS * config.robot_obstacle_inflation_factor()),
            circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a stadium enemy robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, enemy_robot_obstacle_slow)
{
    Point origin(1.0, 1.0);
    // Speed just slightly below the minimum speed for the obstacle to be considered
    // dynamic
    Vector velocity(config.dynamic_enemy_robot_obstacle_min_speed_mps() - 0.01, 0.0);
    Robot robot =
        Robot(4, origin, velocity, Angle::zero(), AngularVelocity::zero(), current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createStadiumEnemyRobotObstacle(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<GeomObstacle<Circle>&>(*obstacle);
        TestUtil::equalWithinTolerance(
            Circle(origin,
                   ROBOT_MAX_RADIUS_METERS * config.robot_obstacle_inflation_factor()),
            circle_obstacle.getGeom());
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "GeomObstacle<Circle>Ptr was not created for a stadium enemy robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, trajectory_robot_obstacle)
{
    Point origin(1.0, 1.0);
    Point end(5.0, 1.0);
    Vector velocity(1.0, 0.0);
    Robot robot =
        Robot(4, origin, velocity, Angle::zero(), AngularVelocity::zero(), current_time);

    TrajectoryPath trajectory(std::make_shared<BangBangTrajectory2D>(
                                  origin, end, velocity, KinematicConstraints(1, 1, 1)),
                              BangBangTrajectory2D::generator);

    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createFromMovingRobot(robot, trajectory);

    try
    {
        auto circle_obstacle = dynamic_cast<TrajectoryObstacle<Circle>&>(*obstacle);
        TestUtil::equalWithinTolerance(
            Circle(origin,
                   ROBOT_MAX_RADIUS_METERS * config.robot_obstacle_inflation_factor()),
            circle_obstacle.getGeom());

        EXPECT_TRUE(obstacle->contains(origin, 0.0));
        EXPECT_FALSE(obstacle->contains(end, 0.0));

        double half_way = trajectory.getTotalTime() / 2;
        EXPECT_TRUE(obstacle->contains(trajectory.getPosition(half_way), half_way));

        double total_duration = trajectory.getTotalTime();
        EXPECT_TRUE(obstacle->contains(end, total_duration));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "TrajectoryObstacle<Circle>Ptr was not created for a robot with trajectory";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, const_velocity_enemy_robot_obstacle)
{
    Point origin(1.0, 1.0);
    Vector velocity(1.0, 0.0);
    Robot robot =
        Robot(4, origin, velocity, Angle::zero(), AngularVelocity::zero(), current_time);
    ObstaclePtr obstacle =
        robot_navigation_obstacle_factory.createConstVelocityEnemyRobotObstacle(robot);

    try
    {
        auto circle_obstacle = dynamic_cast<ConstVelocityObstacle<Circle>&>(*obstacle);
        TestUtil::equalWithinTolerance(
            Circle(origin,
                   ROBOT_MAX_RADIUS_METERS * config.robot_obstacle_inflation_factor()),
            circle_obstacle.getGeom());

        // Verify that the obstacle has moved forward 0.3 seconds into the future
        TestUtil::equalWithinTolerance(obstacle->distance(Point(1.3, 1.0), 0.3), 0.0,
                                       0.001);
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "ConstVelocityObstacle<Circle>Ptr was not created for a const velocity enemy robot";
    }
}

TEST_F(RobotNavigationObstacleFactoryTest, trajectory_obstacle)
{
    Point origin(1.0, 1.0);
    Point destination(4.0, 1.0);
    Vector velocity(1.0, 0.0);

    auto trajectory = std::make_shared<BangBangTrajectory2D>(
        origin, destination, velocity, KinematicConstraints(1.0, 1.0, 1.0));
    TrajectoryPath trajectory_path(trajectory, BangBangTrajectory2D::generator);

    Robot robot =
        Robot(4, origin, velocity, Angle::zero(), AngularVelocity::zero(), current_time);
    ObstaclePtr obstacle = robot_navigation_obstacle_factory.createCircleWithTrajectory(
        Circle(origin, ROBOT_MAX_RADIUS_METERS), trajectory_path);

    try
    {
        auto circle_obstacle = dynamic_cast<TrajectoryObstacle<Circle>&>(*obstacle);
        TestUtil::equalWithinTolerance(
            Circle(origin,
                   ROBOT_MAX_RADIUS_METERS * config.robot_obstacle_inflation_factor()),
            circle_obstacle.getGeom());

        // Verify that the obstacle does move in the future
        EXPECT_FALSE(
            obstacle->contains(origin - Vector(ROBOT_MAX_RADIUS_METERS, 0.0), 0.5));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE()
            << "TrajectoryObstacle<Circle>Ptr was not created for a trajectory obstacle";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, centre_circle)
{
    auto obstacles =
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::CENTER_CIRCLE, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_DEFENSE_AREA, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::ENEMY_DEFENSE_AREA, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::FRIENDLY_HALF, world_ptr);
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
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::ENEMY_HALF, world_ptr);
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

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, ball_placement_stadium)
{
    Ball new_ball               = Ball(Point(0, 0), Vector(0, 0), current_time);
    Point placement_point       = Point(1, 0);
    GameState ball_placement_gs = world_ptr->gameState();
    ball_placement_gs.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    ball_placement_gs.setBallPlacementPoint(placement_point);
    ball_placement_gs.updateBall(new_ball);
    world_ptr->updateBall(new_ball);
    world_ptr->updateGameState(ball_placement_gs);
    auto obstacles =
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE, world_ptr);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Stadium expected(Point(0, 0), Point(1, 0), 0.617);
        auto stadium_obstacle = dynamic_cast<GeomObstacle<Stadium>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, stadium_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }

    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Stadium Obstacle was not created";
    }
}

TEST_F(RobotNavigationObstacleFactoryMotionConstraintTest, ball_placement_rotated)
{
    Ball new_ball               = Ball(Point(0, 0), Vector(0, 0), current_time);
    Point placement_point       = Point(1, 1);
    GameState ball_placement_gs = world_ptr->gameState();
    ball_placement_gs.updateRefereeCommand(RefereeCommand::BALL_PLACEMENT_THEM);
    ball_placement_gs.setBallPlacementPoint(placement_point);
    ball_placement_gs.updateBall(new_ball);
    world_ptr->updateBall(new_ball);
    world_ptr->updateGameState(ball_placement_gs);
    auto obstacles =
        robot_navigation_obstacle_factory.createObstaclesFromMotionConstraint(
            TbotsProto::MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE, world_ptr);
    EXPECT_EQ(1, obstacles.size());
    try
    {
        Stadium expected(Point(0, 0), Point(1, 1), 0.617);
        auto stadium_obstacle = dynamic_cast<GeomObstacle<Stadium>&>(*obstacles[0]);
        EXPECT_TRUE(TestUtil::equalWithinTolerance(expected, stadium_obstacle.getGeom(),
                                                   METERS_PER_MILLIMETER));
    }
    catch (std::bad_cast&)
    {
        ADD_FAILURE() << "Stadium Obstacle was not created";
    }
}
