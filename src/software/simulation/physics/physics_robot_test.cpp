#include "software/simulation/physics/physics_robot.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "shared/2015_robot_constants.h"
#include "shared/constants.h"
#include "software/simulation/physics/box2d_util.h"
#include "software/simulation/physics/physics_ball.h"
#include "software/test_util/equal_within_tolerance.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"

class PhysicsRobotTest : public testing::Test
{
   public:
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    static constexpr int BOX2D_STEP_VELOCITY_ITERATIONS = 5;
    static constexpr int BOX2D_STEP_POSITION_ITERATIONS = 8;

   protected:
    virtual void SetUp()
    {
        b2Vec2 gravity(0, 0);
        world = std::make_shared<b2World>(gravity);
    }

    b2Body* createBallBody(Point position, double radius)
    {
        b2BodyDef ball_body_def;
        ball_body_def.type = b2_dynamicBody;
        ball_body_def.position.Set(static_cast<float>(position.x()),
                                   static_cast<float>(position.y()));
        b2Body* body = world->CreateBody(&ball_body_def);

        b2CircleShape ball_shape;
        ball_shape.m_radius = static_cast<float>(radius);
        b2FixtureDef ball_fixture_def;
        ball_fixture_def.shape = &ball_shape;
        body->CreateFixture(&ball_fixture_def);

        return body;
    }

    void simulateForDuration(const Duration& duration)
    {
        double step_size_seconds = 1.0 / 60.0;
        unsigned int num_steps =
            static_cast<unsigned int>(duration.toSeconds() / step_size_seconds);

        // We have to take lots of small steps because a significant amount of accuracy
        // is lost if we take a single step of 1 second
        for (unsigned int i = 0; i < num_steps; i++)
        {
            world->Step(static_cast<float>(step_size_seconds),
                        BOX2D_STEP_VELOCITY_ITERATIONS, BOX2D_STEP_POSITION_ITERATIONS);
        }
    }

    std::shared_ptr<b2World> world;
    RobotConstants_t robot_constants = create2015RobotConstants();
};

TEST_F(PhysicsRobotTest, test_get_robot_id)
{
    RobotState initial_robot_state(Point(-1.3, 2), Vector(0, 0.15),
                                   Angle::fromDegrees(45),
                                   AngularVelocity::fromDegrees(-170));
    PhysicsRobot physics_robot(3, world, initial_robot_state, robot_constants);

    EXPECT_EQ(3, physics_robot.getRobotId());
}

TEST_F(PhysicsRobotTest, test_get_physics_robot_state_members)
{
    RobotState initial_robot_state(Point(-1.3, 2), Vector(0, 0.15),
                                   Angle::fromDegrees(45),
                                   AngularVelocity::fromDegrees(-170));
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // Most values are stored as floats in Box2D so we can't have perfect comparisons
    EXPECT_LT((Point(-1.3, 2) - physics_robot.position()).length(), 1e-6);
    EXPECT_LT((Vector(0, 0.15) - physics_robot.velocity()).length(), 0.02);
    EXPECT_LT(Angle::fromDegrees(45).minDiff(physics_robot.orientation()),
              Angle::fromDegrees(0.1));
    EXPECT_LT(AngularVelocity::fromDegrees(-170).minDiff(physics_robot.angularVelocity()),
              AngularVelocity::fromDegrees(0.1));
}

TEST_F(PhysicsRobotTest, test_get_robot_state)
{
    RobotState initial_robot_state(Point(-1.3, 2), Vector(0, 0.15),
                                   Angle::fromDegrees(45),
                                   AngularVelocity::fromDegrees(-170));
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);
    RobotState physics_robot_state = physics_robot.getRobotState();

    // Most values are stored as floats in Box2D so we can't have perfect comparisons
    EXPECT_LT((Point(-1.3, 2) - physics_robot_state.position()).length(), 1e-6);
    EXPECT_LT((Vector(0, 0.15) - physics_robot_state.velocity()).length(), 0.02);
    EXPECT_LT(Angle::fromDegrees(45).minDiff(physics_robot_state.orientation()),
              Angle::fromDegrees(0.1));
    EXPECT_LT(
        AngularVelocity::fromDegrees(-170).minDiff(physics_robot_state.angularVelocity()),
        AngularVelocity::fromDegrees(0.1));
}

TEST_F(PhysicsRobotTest, test_robot_added_to_physics_world_on_creation)
{
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());

    EXPECT_EQ(0, world->GetBodyCount());

    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST_F(PhysicsRobotTest, test_physics_robot_is_removed_from_world_when_destroyed)
{
    {
        RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                       AngularVelocity::zero());

        EXPECT_EQ(0, world->GetBodyCount());

        PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

        EXPECT_EQ(1, world->GetBodyCount());
    }

    // Once we leave the above scope the robot is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_left_side_outside_radius)
{
    // Roll an object along the left side of the robot just outside of the robot radius
    // and check it does not collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius  = 0.1;
    auto ball_body = createBallBody(Point(1, ROBOT_MAX_RADIUS_METERS + radius), radius);
    ball_body->SetLinearVelocity({-2, 0});

    simulateForDuration(Duration::fromSeconds(1));

    EXPECT_EQ(b2Vec2(-2, 0), ball_body->GetLinearVelocity());
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_left_side_inside_radius)
{
    // Roll an object along the left side of the robot just inside of the robot radius
    // and check it does collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius = 0.1;
    auto ball_body =
        createBallBody(Point(1, ROBOT_MAX_RADIUS_METERS + radius - 0.005), radius);
    ball_body->SetLinearVelocity({-2, 0});

    simulateForDuration(Duration::fromSeconds(1));

    // Use the cross-product to check if the new velocity vector is rotated
    // clockwise of the old vector
    EXPECT_LT(b2Cross(b2Vec2(-2, 0), ball_body->GetLinearVelocity()), 0);
    EXPECT_NEAR(b2Vec2(-2, 0).Length(), ball_body->GetLinearVelocity().Length(), 0.1);
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_right_side_outside_radius)
{
    // Roll an object along the right side of the robot just outside of the robot radius
    // and check it does not collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius  = 0.1;
    auto ball_body = createBallBody(Point(1, -ROBOT_MAX_RADIUS_METERS - radius), radius);
    ball_body->SetLinearVelocity({-2, 0});

    simulateForDuration(Duration::fromSeconds(1));

    EXPECT_EQ(b2Vec2(-2, 0), ball_body->GetLinearVelocity());
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_right_side_inside_radius)
{
    // Roll an object along the right side of the robot just inside of the robot radius
    // and check it does collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius = 0.1;
    auto ball_body =
        createBallBody(Point(1, -ROBOT_MAX_RADIUS_METERS - radius + 0.005), radius);
    ball_body->SetLinearVelocity({-2, 0});

    simulateForDuration(Duration::fromSeconds(1));

    // Use the cross-product to check if the new velocity vector is rotated
    // counter-clockwise of the old vector
    EXPECT_GT(b2Cross(b2Vec2(-2, 0), ball_body->GetLinearVelocity()), 0);
    EXPECT_NEAR(b2Vec2(-2, 0).Length(), ball_body->GetLinearVelocity().Length(), 0.1);
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_back_side_outside_radius)
{
    // Roll an object along the back side of the robot just outside of the robot radius
    // and check it does not collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius  = 0.1;
    auto ball_body = createBallBody(Point(-ROBOT_MAX_RADIUS_METERS - radius, 1), radius);
    ball_body->SetLinearVelocity({0, -2});

    simulateForDuration(Duration::fromSeconds(1));

    EXPECT_EQ(b2Vec2(0, -2), ball_body->GetLinearVelocity());
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_back_side_inside_radius)
{
    // Roll an object along the back side of the robot just inside of the robot radius
    // and check it does collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius = 0.1;
    auto ball_body =
        createBallBody(Point(-ROBOT_MAX_RADIUS_METERS - radius + 0.005, 1), radius);
    ball_body->SetLinearVelocity({0, -2});

    simulateForDuration(Duration::fromSeconds(1));

    // Use the cross-product to check if the new velocity vector is rotated
    // clockwise of the old vector
    EXPECT_LT(b2Cross(b2Vec2(0, -2), ball_body->GetLinearVelocity()), 0);
    EXPECT_NEAR(b2Vec2(0, -2).Length(), ball_body->GetLinearVelocity().Length(), 0.1);
}

TEST_F(PhysicsRobotTest, test_physics_robot_dimensions_front_side_outside_robot_body)
{
    // Roll an object along the front side of the robot just outside of the robot body
    // and check it does not collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius = 0.1;
    auto ball_body =
        createBallBody(Point(DIST_TO_FRONT_OF_ROBOT_METERS + radius + 0.005, 1), radius);
    ball_body->SetLinearVelocity({0, -2});

    simulateForDuration(Duration::fromSeconds(1));

    EXPECT_EQ(b2Vec2(0, -2), ball_body->GetLinearVelocity());
}

TEST_F(PhysicsRobotTest,
       test_physics_robot_dimensions_front_side_slightly_inside_robot_body)
{
    // Roll an object along the front side of the robot just inside of the body
    // and check it does collide
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    double radius  = 0.1;
    auto ball_body = createBallBody(
        Point(DIST_TO_FRONT_OF_ROBOT_METERS + radius - 0.005, 0.5), radius);
    ball_body->SetLinearVelocity({0, -2});

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);

        // Once the ball has passed y=0 we expect it to have collided with the front of
        // the robot
        if (ball_body->GetPosition().y <= 0)
        {
            // Use the cross-product to check if the new velocity vector is rotated
            // counter-clockwise of the old vector
            EXPECT_GT(b2Cross(b2Vec2(0, -2), ball_body->GetLinearVelocity()), 0);
            EXPECT_NEAR(b2Vec2(0, -2).Length(), ball_body->GetLinearVelocity().Length(),
                        0.1);
        }
    }

    // Use the cross-product to check if the new velocity vector is rotated
    // counter-clockwise of the old vector
    EXPECT_GT(b2Cross(b2Vec2(0, -2), ball_body->GetLinearVelocity()), 0);
    EXPECT_NEAR(b2Vec2(0, -2).Length(), ball_body->GetLinearVelocity().Length(), 0.1);

    // Create a new body and roll it the opposite way to check we collide with the front
    // of the robot from both directions, since it's made of 2 separate pieces with a bap
    // in-between, and we need to make sure we can collide with both pieces
    auto ball_body_2 = createBallBody(
        Point(DIST_TO_FRONT_OF_ROBOT_METERS + radius - 0.005, -0.5), radius);
    ball_body_2->SetLinearVelocity({0, 2});

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);

        // Once the ball has passed y=0 we expect it to have collided with the front of
        // the robot
        if (ball_body->GetPosition().y >= 0)
        {
            // Use the cross-product to check if the new velocity vector is rotated
            // clockwise of the old vector
            EXPECT_LT(b2Cross(b2Vec2(0, 2), ball_body->GetLinearVelocity()), 0);
            EXPECT_NEAR(b2Vec2(0, 2).Length(), ball_body->GetLinearVelocity().Length(),
                        0.1);
        }
    }

    // Use the cross-product to check if the new velocity vector is rotated
    // clockwise of the old vector
    EXPECT_LT(b2Cross(b2Vec2(0, 2), ball_body->GetLinearVelocity()), 0);
    EXPECT_NEAR(b2Vec2(0, 2).Length(), ball_body->GetLinearVelocity().Length(), 0.1);
}

TEST_F(PhysicsRobotTest, test_dribbler_ball_contact_callbacks)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_TRUE(physics_robot.getDribblerBallContactCallbacks().empty());

    bool callback_called = false;
    auto callback        = [&callback_called](PhysicsRobot* robot, PhysicsBall* ball) {
        callback_called = true;
    };

    physics_robot.registerDribblerBallContactCallback(callback);

    ASSERT_EQ(physics_robot.getDribblerBallContactCallbacks().size(), 1);
    physics_robot.getDribblerBallContactCallbacks().at(0)(nullptr, nullptr);
    EXPECT_TRUE(callback_called);
}

TEST_F(PhysicsRobotTest, test_dribbler_ball_start_contact_callbacks)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_TRUE(physics_robot.getDribblerBallStartContactCallbacks().empty());

    bool callback_called = false;
    auto callback        = [&callback_called](PhysicsRobot* robot, PhysicsBall* ball) {
        callback_called = true;
    };

    physics_robot.registerDribblerBallStartContactCallback(callback);

    ASSERT_EQ(physics_robot.getDribblerBallStartContactCallbacks().size(), 1);
    physics_robot.getDribblerBallStartContactCallbacks().at(0)(nullptr, nullptr);
    EXPECT_TRUE(callback_called);
}

TEST_F(PhysicsRobotTest, test_dribbler_ball_end_contact_callbacks)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_TRUE(physics_robot.getDribblerBallEndContactCallbacks().empty());

    bool callback_called = false;
    auto callback        = [&callback_called](PhysicsRobot* robot, PhysicsBall* ball) {
        callback_called = true;
    };

    physics_robot.registerDribblerBallEndContactCallback(callback);

    ASSERT_EQ(physics_robot.getDribblerBallEndContactCallbacks().size(), 1);
    physics_robot.getDribblerBallEndContactCallbacks().at(0)(nullptr, nullptr);
    EXPECT_TRUE(callback_called);
}

enum class RobotWheel
{
    FRONT_LEFT,
    BACK_LEFT,
    BACK_RIGHT,
    FRONT_RIGHT
};

class PhysicsRobotWheelForceTest
    : public ::testing::TestWithParam<
          std::tuple<double, double, double, RobotWheel, double>>
{
   protected:
    RobotConstants_t robot_constants = create2015RobotConstants();
};

TEST_P(PhysicsRobotWheelForceTest, test_wheel_force_creates_angular_velocity)
{
    auto params = GetParam();
    Point robot_position(std::get<0>(params), std::get<1>(params));
    RobotWheel wheel   = std::get<3>(params);
    double wheel_force = std::get<4>(params);

    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::zero(),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    switch (wheel)
    {
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

    world->Step(static_cast<float>(1.0 / 60.0),
                PhysicsRobotTest::BOX2D_STEP_VELOCITY_ITERATIONS,
                PhysicsRobotTest::BOX2D_STEP_POSITION_ITERATIONS);

    auto robot = physics_robot.getRobotState();

    if (wheel_force > 0)
    {
        EXPECT_GT(robot.angularVelocity(), Angle::zero());
    }
    else if (wheel_force < 0)
    {
        EXPECT_LT(robot.angularVelocity(), Angle::zero());
    }
    else
    {
        EXPECT_EQ(robot.angularVelocity(), Angle::zero());
    }
}

INSTANTIATE_TEST_CASE_P(
    All, PhysicsRobotWheelForceTest,
    ::testing::Combine(
        testing::Values(-5.0, 0.0, 5.0),                  // Robot x coordinate
        testing::Values(-5.0, 0.0, 5.0),                  // Robot y coordinate
        testing::Values(0.0, 50.0, 166.0, 201.0, 315.0),  // Robot orientation degrees
        testing::Values(RobotWheel::FRONT_LEFT, RobotWheel::BACK_LEFT,
                        RobotWheel::BACK_RIGHT,
                        RobotWheel::FRONT_RIGHT),  // The wheel to apply force to
        testing::Values(-0.5, 0.5)                 // Wheel force
        ));

TEST_F(PhysicsRobotTest, test_robot_drive_forward_at_angle)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(45),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontRight(0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_GT(robot.velocity().x(), 0.4);
    EXPECT_GT(robot.velocity().y(), 0.4);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_GT(robot.position().x(), 0.3);
    EXPECT_GT(robot.position().y(), 0.3);
    EXPECT_NEAR(robot.orientation().toDegrees(), 45, 1);
}

TEST_F(PhysicsRobotTest, test_robot_drive_forward)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontRight(0.5);

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_GT(robot.velocity().x(), 0.5);
    EXPECT_NEAR(robot.velocity().y(), 0, 1e-5);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_GT(robot.position().x(), 0.4);
    EXPECT_NEAR(robot.position().y(), 0, 1e-5);
    EXPECT_NEAR(robot.orientation().toDegrees(), 0, 1);
}

TEST_F(PhysicsRobotTest, test_robot_drive_backwards)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(0.5);
        physics_robot.applyWheelForceBackLeft(0.5);
        physics_robot.applyWheelForceBackRight(-0.5);
        physics_robot.applyWheelForceFrontRight(-0.5);

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_LT(robot.velocity().x(), -0.5);
    EXPECT_NEAR(robot.velocity().y(), 0, 1e-5);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0, 1);

    EXPECT_LT(robot.position().x(), -0.4);
    EXPECT_NEAR(robot.position().y(), 0, 1e-5);
    EXPECT_NEAR(robot.orientation().toDegrees(), 0, 1);
}

TEST_F(PhysicsRobotTest, test_robot_spin_clockwise)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(-0.5);
        physics_robot.applyWheelForceBackLeft(-0.5);
        physics_robot.applyWheelForceBackRight(-0.5);
        physics_robot.applyWheelForceFrontRight(-0.5);

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    // Because the robot's center of mass is not perfect we expect it to have drifted a
    // little bit while spinning
    EXPECT_LT((robot.position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_LT(robot.angularVelocity(), AngularVelocity::fromRadians(-20));
}

TEST_F(PhysicsRobotTest, test_robot_spin_counterclockwise)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyWheelForceFrontLeft(0.5);
        physics_robot.applyWheelForceBackLeft(0.5);
        physics_robot.applyWheelForceBackRight(0.5);
        physics_robot.applyWheelForceFrontRight(0.5);

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    // Because the robot's center of mass is not perfect we expect it to have drifted a
    // little bit while spinning
    EXPECT_LT((robot.position() - Point(0, 0)).length(), 0.05);
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.05);
    EXPECT_GT(robot.angularVelocity(), AngularVelocity::fromRadians(20));
}

TEST_F(PhysicsRobotTest, test_get_motor_speeds_when_robot_not_moving)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_EQ(physics_robot.getMotorSpeedFrontLeft(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedBackLeft(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedBackRight(), 0.0);
    EXPECT_EQ(physics_robot.getMotorSpeedFrontRight(), 0.0);
}

TEST_F(PhysicsRobotTest, test_get_motor_speeds_when_robot_moving_forwards)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(1, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_LT(physics_robot.getMotorSpeedFrontLeft(), -1.0);
    EXPECT_LT(physics_robot.getMotorSpeedBackLeft(), -1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
    EXPECT_LT(physics_robot.getMotorSpeedFrontLeft(),
              physics_robot.getMotorSpeedBackLeft());
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(),
              physics_robot.getMotorSpeedBackRight());
    EXPECT_EQ(physics_robot.getMotorSpeedFrontLeft(),
              -physics_robot.getMotorSpeedFrontRight());
    EXPECT_EQ(physics_robot.getMotorSpeedBackLeft(),
              -physics_robot.getMotorSpeedBackRight());
}

TEST_F(PhysicsRobotTest, test_get_motor_speeds_when_robot_moving_along_wheel_axis)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is
    // perpendicular to the direction of motion, and we don't expect it to be spinning
    Vector robot_velocity =
        Vector::createFromAngle(Angle::fromDegrees(ANGLE_TO_ROBOT_FRONT_WHEELS_DEG));
    RobotState initial_robot_state(Point(0, 0), robot_velocity, Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_NEAR(physics_robot.getMotorSpeedFrontLeft(), 0.0, 0.1);
    EXPECT_LT(physics_robot.getMotorSpeedBackLeft(), -1.0);
    EXPECT_LT(physics_robot.getMotorSpeedBackRight(), -0.1);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
}

TEST_F(PhysicsRobotTest, test_get_motor_speeds_when_robot_spinning)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Move along the axis of the front-left wheel. This means the front-left wheel is
    // perpendicular to the direction of motion, and we don't expect it to be spinning
    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::threeQuarter());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    EXPECT_GT(physics_robot.getMotorSpeedFrontLeft(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackLeft(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedBackRight(), 1.0);
    EXPECT_GT(physics_robot.getMotorSpeedFrontRight(), 1.0);
}

TEST_F(PhysicsRobotTest,
       test_brake_motors_when_robot_spinning_with_positive_angular_velocity)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::threeQuarter());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST_F(PhysicsRobotTest,
       test_brake_motors_when_robot_spinning_with_negative_angular_velocity)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::fromDegrees(-2 * 360));
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST_F(PhysicsRobotTest, test_brake_motors_when_robot_moving_linearly)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(2.5, 1.0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 240; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST_F(PhysicsRobotTest, test_brake_motors_when_robot_moving_and_spinning)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(2.5, 1.0), Angle::fromDegrees(0),
                                   AngularVelocity::threeQuarter());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 240; i++)
    {
        physics_robot.brakeMotorFrontLeft();
        physics_robot.brakeMotorBackLeft();
        physics_robot.brakeMotorBackRight();
        physics_robot.brakeMotorFrontRight();

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_LT((robot.velocity() - Vector(0, 0)).length(), 0.01);
    EXPECT_NEAR(robot.angularVelocity().toDegrees(), 0.0, 1);
}

TEST_F(PhysicsRobotTest, test_apply_force_to_center)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    RobotState initial_robot_state(Point(0, 0), Vector(0, 0), Angle::fromDegrees(0),
                                   AngularVelocity::zero());
    PhysicsRobot physics_robot(0, world, initial_robot_state, robot_constants);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_robot.applyForceToCenterOfMass(Vector(1, 1));

        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto robot = physics_robot.getRobotState();
    EXPECT_TRUE(TestUtil::equalWithinTolerance(robot.velocity().orientation(),
                                               Vector(1, 1).orientation(),
                                               Angle::fromDegrees(2)));
    // Damping and other factors mean we can't assert the robot's exact speed, so
    // we just check it got moving beyond a low threshold
    EXPECT_GT(robot.velocity().length(), 0.2);
}
