#include "software/backend/simulation/physics/physics_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "shared/constants.h"
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/test_util/test_util.h"
#include "software/world/robot.h"

TEST(PhysicsRobotTest, test_get_robot_with_timestamp)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);
    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(3.3));

    EXPECT_EQ(robot_parameter.position(), robot.position());
    EXPECT_EQ(Timestamp::fromSeconds(3.3), robot.lastUpdateTimestamp());
}

TEST(PhysicsRobotTest, test_get_mass) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 2.421);

    EXPECT_NEAR(physics_robot.getMassKg(), 2.421, 1e-6);
}

TEST(PhysicsRobotTest, test_robot_added_to_physics_world_on_creation)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world->GetBodyCount());

    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST(PhysicsRobotTest, test_physics_robot_is_removed_from_world_when_destroyed)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    {
        Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                              AngularVelocity::zero(), Timestamp::fromSeconds(0));

        EXPECT_EQ(0, world->GetBodyCount());

        PhysicsRobot physics_robot(world, robot_parameter, 1.0);

        EXPECT_EQ(1, world->GetBodyCount());
    }

    // Once we leave the above scope the robot is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

// Roll the ball along the left side of the robot just outside of the robot radius
// and check it does not collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_outside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(Point(1, ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS),
                        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(ball.velocity().orientation().minDiff(Angle::half()).toDegrees(), 0.1);
}

// Roll the ball along the left side of the robot just inside of the robot radius
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_inside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(
        Point(1, ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS - 0.005), Vector(-2, 0),
        Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(ball.velocity().orientation().minDiff(Angle::half()).toDegrees(), 0.1);
}

// Roll the ball along the right side of the robot just outside of the robot radius
// and check it does not collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_right_side_outside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(Point(1, -ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS),
                        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(ball.velocity().orientation().minDiff(Angle::half()).toDegrees(), 0.1);
}

// Roll the ball along the right side of the robot just inside of the robot radius
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_right_side_inside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(
        Point(1, -ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS + 0.005),
        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(ball.velocity().orientation().minDiff(Angle::half()).toDegrees(), 0.1);
}

// Roll the ball along the back side of the robot just outside of the robot radius
// and check it does not collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_back_side_outside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(Point(-ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS, 1),
                        Vector(0, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(ball.velocity().orientation().minDiff(Angle::threeQuarter()).toDegrees(),
              0.1);
}

// Roll the ball along the back side of the robot just inside of the robot radius
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_back_side_inside_radius)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(
        Point(-ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS + 0.005, 1),
        Vector(0, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(ball.velocity().orientation().minDiff(Angle::threeQuarter()).toDegrees(),
              0.1);
}

// Roll the ball along the front side of the robot just in front of the chicker
// and check it does not collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_front_side_in_front_of_chicker)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(
        Point(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS + 0.003, 1),
        Vector(0, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(ball.velocity().orientation().minDiff(Angle::threeQuarter()).toDegrees(),
              0.1);
}

// Roll the ball along the front side of the robot just behind the chicker
// and check it does collide
TEST(PhysicsRobotTest, test_physics_robot_dimensions_front_side_behind_chicker)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(
        Point(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS - 0.005, 1),
        Vector(0, -2), Timestamp::fromSeconds(0));
    PhysicsBall physics_ball(world, ball_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(ball.velocity().orientation().minDiff(Angle::threeQuarter()).toDegrees(),
              0.1);
}
