#include "software/simulation/physics/physics_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "shared/constants.h"
#include "software/test_util/test_util.h"
#include "software/time/duration.h"

class PhysicsBallTest : public testing::Test
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
        world          = std::make_shared<b2World>(gravity);
        physics_config = std::make_shared<const PhysicsConfig>();
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
    std::shared_ptr<const PhysicsConfig> physics_config;
};

TEST_F(PhysicsBallTest, test_get_position)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_LT((initial_ball_state.position() - physics_ball.position()).length(), 1e-7);
}

TEST_F(PhysicsBallTest, test_get_velocity)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_LT((initial_ball_state.velocity() - physics_ball.velocity()).length(), 1e-7);
}

TEST_F(PhysicsBallTest, test_get_momentum)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    Vector expected_momentum = Vector(1, -2) * 1.0;
    EXPECT_LT((expected_momentum - physics_ball.momentum()).length(), 1e-6);
}

TEST_F(PhysicsBallTest, test_get_mass)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_FLOAT_EQ(1.0f, physics_ball.massKg());
}

TEST_F(PhysicsBallTest, test_get_ball_state)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);
    auto ball_state   = physics_ball.getBallState();

    EXPECT_LT((initial_ball_state.position() - ball_state.position()).length(), 1e-7);
    EXPECT_LT((initial_ball_state.velocity() - ball_state.velocity()).length(), 1e-7);
}

TEST_F(PhysicsBallTest, test_ball_added_to_physics_world_on_creation)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST_F(PhysicsBallTest, test_physics_ball_is_removed_from_world_when_destroyed)
{
    {
        BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

        EXPECT_EQ(1, world->GetBodyCount());
    }

    // Once we leave the above scope the ball is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

TEST_F(PhysicsBallTest, test_ball_velocity_and_position_updates_during_simulation_step)
{
    BallState initial_ball_state(Point(1, -1), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    simulateForDuration(Duration::fromSeconds(1));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Point(2, -3) - ball.position()).length(), 0.01);
    EXPECT_LT((Vector(1, -2) - ball.velocity()).length(), 1e-5);
}

TEST_F(PhysicsBallTest,
       test_ball_acceleration_and_velocity_updates_during_simulation_step)
{
    b2Vec2 gravity(3, -0.5);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    for (unsigned int i = 0; i < 60; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(3, -0.5) - ball.velocity()).length(), 0.01);
}

TEST_F(PhysicsBallTest, test_ball_reverses_direction_after_object_collision)
{
    // Create a wall object for the ball to bounce off of
    b2BodyDef wall_body_def;
    wall_body_def.type = b2_staticBody;
    wall_body_def.position.Set(1.5, 0.0);
    wall_body_def.angle = 0.0;
    b2Body* wall_body   = world->CreateBody(&wall_body_def);

    b2PolygonShape wall_shape;
    wall_shape.SetAsBox(1, 1);

    b2FixtureDef wall_fixture_def;
    wall_fixture_def.shape = &wall_shape;
    // Perfectly elastic collisions and no friction
    wall_fixture_def.restitution = 1.0;
    wall_fixture_def.friction    = 0.0;
    wall_body->CreateFixture(&wall_fixture_def);

    BallState initial_ball_state(Point(0, 0), Vector(0.5, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    simulateForDuration(Duration::fromSeconds(1));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(-0.5, 0.0) - ball.velocity()).length(), 1e-7);
}

TEST_F(PhysicsBallTest, test_ball_changes_direction_after_object_deflection)
{
    // Create a box angled 45 degrees so the ball is deflected downwards
    // The box is slightly raised in the y-axis so the ball doesn't perfectly
    // hit the corner
    b2BodyDef wall_body_def;
    wall_body_def.type = b2_staticBody;
    wall_body_def.position.Set(2.0, 0.5);
    wall_body_def.angle = static_cast<float>(M_PI_4);
    b2Body* wall_body   = world->CreateBody(&wall_body_def);

    b2PolygonShape wall_shape;
    wall_shape.SetAsBox(1, 1);

    b2FixtureDef wall_fixture_def;
    wall_fixture_def.shape = &wall_shape;
    // Perfectly elastic collisions and no friction
    wall_fixture_def.restitution = 1.0;
    wall_fixture_def.friction    = 0.0;
    wall_body->CreateFixture(&wall_fixture_def);

    BallState initial_ball_state(Point(0, 0), Vector(1.0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    simulateForDuration(Duration::fromSeconds(2));

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(0.0, -1.0) - ball.velocity()).length(), 1e-5);
}

TEST_F(PhysicsBallTest, test_apply_force_to_stationary_ball)
{
    // Apply force to a stationary ball
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(1, 2));
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_apply_force_to_reverse_direction_of_moving_ball)
{
    // Apply force against a moving ball to reverse its direction
    BallState initial_ball_state(Point(0, 0), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(-2, 4));
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(-1, 2)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_apply_force_to_change_direction_of_moving_ball)
{
    // Apply force to change the direction of a moving ball
    BallState initial_ball_state(Point(0, 0), Vector(1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(0, -1));
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, -1)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_apply_impulse_to_stationary_ball)
{
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    physics_ball.applyImpulse(Vector(1, 2));
    world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                BOX2D_STEP_POSITION_ITERATIONS);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_apply_impulse_to_stop_moving_ball)
{
    BallState initial_ball_state(Point(0, 0), Vector(2, -1));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    physics_ball.applyImpulse(Vector(-2, 1));
    world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                BOX2D_STEP_POSITION_ITERATIONS);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(0, 0)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_apply_impulse_to_change_direction_of_moving_ball)
{
    BallState initial_ball_state(Point(0, 0), Vector(-3, -0.5));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    physics_ball.applyImpulse(Vector(2, 0.5));
    world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                BOX2D_STEP_POSITION_ITERATIONS);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(-1, 0)).length(), 0.05);
}

TEST_F(PhysicsBallTest, test_set_ball_in_flight_without_collisions)
{
    BallState initial_ball_state(Point(0, 0), Vector(1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.setInFlightForDistance(1.0f, Angle::fromDegrees(45));
    EXPECT_TRUE(physics_ball.isInFlight());

    for (unsigned int i = 0; i < 120; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
        physics_ball.updateIsInFlight();
        auto ball = physics_ball.getBallState();
        if (ball.position().x() < 1.0)
        {
            EXPECT_TRUE(physics_ball.isInFlight());
        }
        else
        {
            EXPECT_FALSE(physics_ball.isInFlight());
        }
    }

    EXPECT_FALSE(physics_ball.isInFlight());
}

TEST_F(PhysicsBallTest, test_set_ball_in_flight_with_collisions)
{
    BallState initial_ball_state(Point(0, 0), Vector(1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    // Create a 1x1 box centered at (2, 0) in the world
    b2BodyDef obstacle_body_def;
    obstacle_body_def.type = b2_dynamicBody;
    obstacle_body_def.position.Set(2, 0);
    b2Body* obstacle_body = world->CreateBody(&obstacle_body_def);
    b2PolygonShape obstacle_shape;
    obstacle_shape.SetAsBox(0.5, 0.5);  // A 1x1 box
    b2FixtureDef obstacle_fixture_def;
    obstacle_fixture_def.shape = &obstacle_shape;
    // Make the obstacle a sensor so it does not physically block the ball
    obstacle_fixture_def.isSensor = true;
    obstacle_body->CreateFixture(&obstacle_fixture_def);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.setInFlightForDistance(2.0f, Angle::fromDegrees(45));
    EXPECT_TRUE(physics_ball.isInFlight());

    for (unsigned int i = 0; i < 300; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
        physics_ball.updateIsInFlight();

        // The ball should be in flight until it has passed the obstacle box. Normally
        // the ball would "land" (no longer be in flight) when it reaches the center
        // of the obstacle, but we expect it to remain in flight until it has passed
        // the obstacle. This simulates the ball landing "on top" of the obstacle
        // and rolling off.
        auto ball = physics_ball.getBallState();
        // Polygons have a "skin" and some slop in Box2D so the ball may move a bit
        // further than expected before the collision stops and isInFlight() returns
        // false, but because chipping isn't perfect in the real world we only need this
        // to be close enough
        if (ball.position().x() - BALL_MAX_RADIUS_METERS <= 2.52)
        {
            EXPECT_TRUE(physics_ball.isInFlight());
        }
        else
        {
            EXPECT_FALSE(physics_ball.isInFlight());
        }
    }

    auto ball = physics_ball.getBallState();
    EXPECT_GT(ball.position().x() - BALL_MAX_RADIUS_METERS, 2.52);
    EXPECT_FALSE(physics_ball.isInFlight());
}

TEST_F(PhysicsBallTest, get_height_when_ball_not_in_flight)
{
    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_DOUBLE_EQ(0.0, physics_ball.getBallState().distanceFromGround());
}

TEST_F(PhysicsBallTest, get_height_when_ball_in_flight)
{
    BallState initial_ball_state(Point(0, 0), Vector(1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.setInFlightForDistance(2.0f, Angle::fromDegrees(45));
    EXPECT_TRUE(physics_ball.isInFlight());
    EXPECT_DOUBLE_EQ(0.0, physics_ball.getBallState().distanceFromGround());

    for (unsigned int i = 0; i < 150; i++)
    {
        world->Step(static_cast<float>(1.0 / 60.0), BOX2D_STEP_VELOCITY_ITERATIONS,
                    BOX2D_STEP_POSITION_ITERATIONS);
        physics_ball.updateIsInFlight();
        auto ball                                = physics_ball.getBallState();
        double expected_max_distance_grom_ground = 0.5;
        if (ball.position().x() < 0.95)
        {
            EXPECT_GT(ball.distanceFromGround(), 0.0);
            EXPECT_LT(ball.distanceFromGround(), expected_max_distance_grom_ground);
        }
        else if (ball.position().x() < 1.05)
        {
            EXPECT_NEAR(expected_max_distance_grom_ground, ball.distanceFromGround(),
                        0.05);
        }
        else if (ball.position().x() <= 2.0)
        {
            EXPECT_GT(ball.distanceFromGround(), 0.0);
            EXPECT_LT(ball.distanceFromGround(), expected_max_distance_grom_ground);
        }
        else
        {
            EXPECT_DOUBLE_EQ(0.0, ball.distanceFromGround());
        }
    }
}

class PhysicsBallFrictionTest : public PhysicsBallTest
{
   protected:
    virtual void SetUp()
    {
        PhysicsBallTest::SetUp();
        physics_config = std::make_shared<PhysicsConfig>();
        physics_config->getMutableSlidingFrictionAcceleration()->setValue(5.0);
        physics_config->getMutableRollingFrictionAcceleration()->setValue(0.5);
    }

    std::shared_ptr<PhysicsConfig> physics_config;
};

TEST_F(PhysicsBallFrictionTest, test_no_friction_kick)
{
    Point position(3, 5);
    Vector velocity(1, -1);
    BallState initial_ball_state(position, velocity);
    physics_config->getMutableSlidingFrictionAcceleration()->setValue(0);
    physics_config->getMutableRollingFrictionAcceleration()->setValue(0);
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);
    physics_ball.setInitialKickSpeed(velocity.length());
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.0));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(velocity, physics_ball.velocity(), 0.01));
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.5));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(velocity, physics_ball.velocity(), 0.01));
}

TEST_F(PhysicsBallFrictionTest, test_rolling_friction)
{
    Point position(3, 5);
    Vector velocity(3, -2);

    BallState initial_ball_state(position, velocity);
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);
    physics_ball.setInitialKickSpeed(velocity.length());
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.0));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Vector(1.81, -1.21),
                                               physics_ball.velocity(), 0.01));
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.5));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Vector(1.19, -0.79),
                                               physics_ball.velocity(), 0.01));
}

TEST_F(PhysicsBallFrictionTest, test_sliding_friction)
{
    Point position(3, 5);
    Vector velocity(3, -2);

    BallState initial_ball_state(position, velocity);
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, physics_config);
    physics_ball.setInitialKickSpeed(velocity.length());
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.0));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Vector(1.81, -1.21),
                                               physics_ball.velocity(), 0.01));
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(1.5));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Vector(1.19, -0.79),
                                               physics_ball.velocity(), 0.01));
    physics_ball.applyBallFrictionModel(Duration::fromSeconds(2.6));
    EXPECT_TRUE(TestUtil::equalWithinTolerance(Vector(0.11, -0.07),
                                               physics_ball.velocity(), 0.01));
}
