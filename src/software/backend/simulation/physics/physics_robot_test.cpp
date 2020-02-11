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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_outside_radius)
{
    // Roll the ball along the left side of the robot just outside of the robot radius
    // and check it does not collide

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(Point(1, ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS),
                        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_left_side_inside_radius)
{
    // Roll the ball along the left side of the robot just inside of the robot radius
    // and check it does collide

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(
        Point(1, ROBOT_MAX_RADIUS_METERS + BALL_MAX_RADIUS_METERS - 0.005), Vector(-2, 0),
        Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_right_side_outside_radius)
{
    // Roll the ball along the right side of the robot just outside of the robot radius
    // and check it does not collide

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(Point(1, -ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS),
                        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_right_side_inside_radius)
{
    // Roll the ball along the right side of the robot just inside of the robot radius
    // and check it does collide

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    Ball ball_parameter(
        Point(1, -ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS + 0.005),
        Vector(-2, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_back_side_outside_radius)
{
    // Roll the ball along the back side of the robot just outside of the robot radius
    // and check it does not collide

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // The ball should pass right next to the robot without colliding, so should not
    // change direction
    Ball ball_parameter(Point(-ROBOT_MAX_RADIUS_METERS - BALL_MAX_RADIUS_METERS, 1),
                        Vector(0, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_back_side_inside_radius)
{
    // Roll the ball along the back side of the robot just inside of the robot radius
    // and check it does collide

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
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_front_side_in_front_of_chicker)
{
    // Roll the ball along the front side of the robot just in front of the chicker
    // and check it does not collide

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
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_physics_robot_dimensions_front_side_behind_chicker)
{
    // Roll the ball along the front side of the robot just behind the chicker
    // and check it does collide

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
    PhysicsBall physics_ball(world, ball_parameter, 1.0, 9.8);

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

TEST(PhysicsRobotTest, test_dribbler_ball_contact_callbacks) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_TRUE(physics_robot.getDribblerBallContactCallbacks().empty());

    bool callback_called = false;
    auto callback = [&callback_called](PhysicsRobot* robot, PhysicsBall* ball) {
        callback_called = true;
    };

    physics_robot.registerDribblerBallContactCallback(callback);

    ASSERT_EQ(physics_robot.getDribblerBallContactCallbacks().size(), 1);
    physics_robot.getDribblerBallContactCallbacks().at(0)(nullptr, nullptr);
    EXPECT_TRUE(callback_called);
}

TEST(PhysicsRobotTest, test_chicker_ball_contact_callbacks) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_TRUE(physics_robot.getChickerBallContactCallbacks().empty());

    bool callback_called = false;
    auto callback = [&callback_called](PhysicsRobot* robot, PhysicsBall* ball) {
        callback_called = true;
    };

    physics_robot.registerChickerBallContactCallback(callback);

    ASSERT_EQ(physics_robot.getChickerBallContactCallbacks().size(), 1);
    physics_robot.getChickerBallContactCallbacks().at(0)(nullptr, nullptr);
    EXPECT_TRUE(callback_called);
}

enum RobotWheel {
    FRONT_LEFT,
    BACK_LEFT,
    BACK_RIGHT,
    FRONT_RIGHT
};

class PhysicsRobotWheelForceTest :
public ::testing::TestWithParam<std::tuple<double, double, double, RobotWheel, double>> {
};

TEST_P(PhysicsRobotWheelForceTest, test_wheel_force_creates_angular_velocity) {
    auto params = GetParam();
    Point robot_position(std::get<0>(params), std::get<1>(params));
    Angle robot_orientation = Angle::fromDegrees(std::get<2>(params));
    RobotWheel wheel = std::get<3>(params);
    double wheel_force = std::get<4>(params);

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, robot_position, Vector(0, 0), robot_orientation,
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    switch(wheel) {
        case RobotWheel::FRONT_LEFT:
            physics_robot.applyWheelForceFrontLeft(wheel_force);
            break;
        case RobotWheel::BACK_LEFT:
            physics_robot.applyWheelForceBackLeft(wheel_force);
            break;
        case RobotWheel::BACK_RIGHT:
            physics_robot.applyWheelForceBackRight(wheel_force);
            break;
        case RobotWheel::FRONT_RIGHT:
            physics_robot.applyWheelForceFrontRight(wheel_force);
            break;
        default:
            ADD_FAILURE() << "Error invalid wheel" << std::endl;
            break;
    }

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));

    if(wheel_force > 0) {
        EXPECT_GT(robot.angularVelocity(), Angle::zero());
    }else if(wheel_force < 0) {
        EXPECT_LT(robot.angularVelocity(), Angle::zero());
    }else {
        EXPECT_EQ(robot.angularVelocity(), Angle::zero());
    }
}

INSTANTIATE_TEST_CASE_P(All,
        PhysicsRobotWheelForceTest,
        ::testing::Combine(
                testing::Values(-5.0, 0.0, 5.0),                 // Robot x coordinate
                testing::Values(-5.0, 0.0, 5.0),                 // Robot y coordinate
                testing::Values(0.0, 50.0, 166.0, 201.0, 315.0), // Robot orientation degrees
                testing::Values(
                        RobotWheel::FRONT_LEFT,
                        RobotWheel::BACK_LEFT,
                        RobotWheel::BACK_RIGHT,
                        RobotWheel::FRONT_RIGHT
                        ),                                       // The wheel to apply force to
                testing::Values(-0.5, 0.5)                       // Wheel force
                ));

TEST(PhysicsRobotTest, test_positive_front_left_wheel_force_creates_positive_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceFrontLeft(1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_positive_back_left_wheel_force_creates_positive_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceBackLeft(1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_positive_back_right_wheel_force_creates_positive_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(1, 1), Vector(0, 0), Angle::fromDegrees(45),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceBackRight(1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_positive_front_right_wheel_force_creates_positive_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceFrontRight(1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_negative_front_left_wheel_force_creates_negative_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceFrontLeft(-1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_negative_back_left_wheel_force_creates_negative_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceBackLeft(-1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_negative_back_right_wheel_force_creates_negative_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceBackRight(-1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_negative_front_right_wheel_force_creates_negative_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    physics_robot.applyWheelForceFrontRight(-1);

    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(robot.angularVelocity(), Angle::zero());
}

TEST(PhysicsRobotTest, test_robot_drive_forward_at_angle) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0),
            Angle::fromDegrees(45),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontRight(0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.velocity().x(), 0.5);
    EXPECT_GT(robot.velocity().y(), 0.5);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_GT(robot.position().x(), 0.3);
    EXPECT_GT(robot.position().y(), 0.3);
    EXPECT_NEAR(robot.orientation().toDegrees(), 45, 1);
}

TEST(PhysicsRobotTest, test_robot_drive_forward) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontRight(0.5);

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(robot.velocity().x(), 1.0);
    EXPECT_NEAR(robot.velocity().y(), 0, 1e-5);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_GT(robot.position().x(), 0.5);
    EXPECT_NEAR(robot.position().y(), 0, 1e-5);
    EXPECT_NEAR(robot.orientation().toDegrees(), 0, 1);
}

TEST(PhysicsRobotTest, test_robot_drive_backwards) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(0.5);
        physics_robot.applyWheelForceBackLeft(0.5);
        physics_robot.applyWheelForceBackRight(-0.5);
        physics_robot.applyWheelForceFrontRight(-0.5);

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT(robot.velocity().x(), -1.0);
    EXPECT_NEAR(robot.velocity().y(), 0, 1e-5);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_LT(robot.position().x(), -0.5);
    EXPECT_NEAR(robot.position().y(), 0, 1e-5);
    EXPECT_NEAR(robot.orientation().toDegrees(), 0, 1);
}

TEST(PhysicsRobotTest, test_robot_spin_clockwise) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);
        physics_robot.applyWheelForceBackRight(-0.5);
        physics_robot.applyWheelForceFrontRight(-0.5);

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    // Because the robot's center of mass is not perfect we expect it to have drifted a little bit
    // while spinning
    EXPECT_LT((robot.position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_LT(robot.angularVelocity(), AngularVelocity::fromRadians(-30));
}

TEST(PhysicsRobotTest, test_robot_spin_counterclockwise) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(0.5);
        physics_robot.applyWheelForceBackLeft(0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontRight(0.5);

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    // Because the robot's center of mass is not perfect we expect it to have drifted a little bit
    // while spinning
    EXPECT_LT((robot.position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_GT(robot.angularVelocity(), AngularVelocity::fromRadians(30));
}

TEST(PhysicsRobotTest, test_get_motor_speeds_when_robot_not_moving) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_EQ(physics_robot.getMotorSpeedFrontLeft(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedBackLeft(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedBackRight(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedFrontRight(), 0.0);
}

TEST(PhysicsRobotTest, test_get_motor_speeds_when_robot_moving_forwards) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Robot robot_parameter(0, Point(0, 0), Vector(1, 0), Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_LT(physics_robot.getMotorSpeedFrontLeft(), -1.0);
    EXPECT_LT(physics_robot.getMotorSpeedBackLeft(), -1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
    EXPECT_LT(physics_robot.getMotorSpeedFrontLeft(), physics_robot.getMotorSpeedBackLeft());
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), physics_robot.getMotorSpeedBackRight());
    EXPECT_EQ(physics_robot.getMotorSpeedFrontLeft(), -physics_robot.getMotorSpeedFrontRight());
    EXPECT_EQ(physics_robot.getMotorSpeedBackLeft(), -physics_robot.getMotorSpeedBackRight());
}

TEST(PhysicsRobotTest, test_get_motor_speeds_when_robot_moving_along_wheel_axis) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Vector robot_velocity = Vector::createFromAngle(Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG));
    Robot robot_parameter(0, Point(0, 0), robot_velocity, Angle::zero(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_NEAR(physics_robot.getMotorSpeedFrontLeft(), 0.0, 0.1);
    EXPECT_LT(physics_robot.getMotorSpeedBackLeft(), -1.0);
    EXPECT_LT(physics_robot.getMotorSpeedBackRight(), -0.1);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
}

TEST(PhysicsRobotTest, test_get_motor_speeds_when_robot_spinning) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    EXPECT_GT(physics_robot.getMotorSpeedFrontLeft(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackLeft(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
}

TEST(PhysicsRobotTest, test_brake_motors_when_robot_spinning_with_positive_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::threeQuarter(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST(PhysicsRobotTest, test_brake_motors_when_robot_spinning_with_negative_angular_velocity) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Robot robot_parameter(0, Point(0, 0), Vector(0, 0), Angle::zero(),
                          AngularVelocity::fromDegrees(-2 * 360), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST(PhysicsRobotTest, test_brake_motors_when_robot_moving_linearly) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Robot robot_parameter(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(),
                          AngularVelocity::zero(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 240; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST(PhysicsRobotTest, test_brake_motors_when_robot_moving_and_spinning) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is perpendicular
    // to the direction of motion, and we don't expect it to be spinning
    Robot robot_parameter(0, Point(0, 0), Vector(2.5, 1.0), Angle::threeQuarter(),
                          AngularVelocity::full(), Timestamp::fromSeconds(0));
    PhysicsRobot physics_robot(world, robot_parameter, 1.0);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 240; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto robot = physics_robot.getRobotWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}
