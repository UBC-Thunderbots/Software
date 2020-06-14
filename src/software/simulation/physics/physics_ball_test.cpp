#include "software/simulation/physics/physics_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "shared/constants.h"

TEST(PhysicsBallTest, test_get_position)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    EXPECT_LT((initial_ball_state.position() - physics_ball.position()).length(), 1e-7);
}

TEST(PhysicsBallTest, test_get_velocity)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    EXPECT_LT((initial_ball_state.velocity() - physics_ball.velocity()).length(), 1e-7);
}

TEST(PhysicsBallTest, test_get_ball_state)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);
    auto ball_state   = physics_ball.getBallState();

    EXPECT_LT((initial_ball_state.position() - ball_state.position()).length(), 1e-7);
    EXPECT_LT((initial_ball_state.velocity() - ball_state.velocity()).length(), 1e-7);
}

TEST(PhysicsBallTest, test_ball_added_to_physics_world_on_creation)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST(PhysicsBallTest, test_physics_ball_is_removed_from_world_when_destroyed)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    {
        BallState initial_ball_state(Point(0.1, -0.04), Vector(1, -2));

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

        EXPECT_EQ(1, world->GetBodyCount());
    }

    // Once we leave the above scope the ball is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

TEST(PhysicsBallTest, test_ball_velocity_and_position_updates_during_simulation_step)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(1, -1), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Point(2, -3) - ball.position()).length(), 0.01);
    EXPECT_LT((Vector(1, -2) - ball.velocity()).length(), 1e-5);
}

TEST(PhysicsBallTest, test_ball_acceleration_and_velocity_updates_during_simulation_step)
{
    b2Vec2 gravity(3, -0.5);
    auto world = std::make_shared<b2World>(gravity);

    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(3, -0.5) - ball.velocity()).length(), 0.01);
}

TEST(PhysicsBallTest, test_ball_reverses_direction_after_object_collision)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

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
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 120; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(-0.5, 0.0) - ball.velocity()).length(), 1e-7);
}

TEST(PhysicsBallTest, test_ball_changes_direction_after_object_deflection)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    // Create a box angled 45 degrees so the ball is deflected downwards
    // The box is slightly raised in the y-axis so the ball doesn't perfectly
    // hit the corner
    b2BodyDef wall_body_def;
    wall_body_def.type = b2_staticBody;
    wall_body_def.position.Set(2.0, 0.5);
    wall_body_def.angle = M_PI_4;
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
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 120; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((Vector(0.0, -1.0) - ball.velocity()).length(), 1e-5);
}

TEST(PhysicsBallTest, test_apply_force_to_stationary_ball)
{
    // Apply force to a stationary ball
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(1, 2));
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_force_to_reverse_direction_of_moving_ball)
{
    // Apply force against a moving ball to reverse its direction
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(1, -2));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(-2, 4));
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(-1, 2)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_force_to_change_direction_of_moving_ball)
{
    // Apply force to change the direction of a moving ball
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        physics_ball.applyForce(Vector(0, -1));
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, -1)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_impulse_to_stationary_ball)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    physics_ball.applyImpulse(Vector(1, 2));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_impulse_to_stop_moving_ball)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(2, -1));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    physics_ball.applyImpulse(Vector(-2, 1));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(0, 0)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_impulse_to_change_direction_of_moving_ball)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(-3, -0.5));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    physics_ball.applyImpulse(Vector(2, 0.5));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);

    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(-1, 0)).length(), 0.05);
}

TEST(PhysicsBallTest, test_kick_stationary_ball)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 0.44, 9.8);

    physics_ball.kick(Vector(-2, 3));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);
    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(-2, 3)).length(), 0.05);
}

TEST(PhysicsBallTest, test_kick_moving_ball)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(-1, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 0.44, 9.8);

    physics_ball.kick(Vector(4, 0));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);
    auto ball = physics_ball.getBallState();
    EXPECT_LT((ball.velocity() - Vector(3, 0)).length(), 0.05);
}

TEST(PhysicsBallTest, test_chip_stationary_ball_without_collisions)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.chip(Vector(1, 0));
    EXPECT_TRUE(physics_ball.isInFlight());

    for (unsigned int i = 0; i < 120; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
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

TEST(PhysicsBallTest, test_chip_ball_with_collisions)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    BallState initial_ball_state(Point(0, 0), Vector(0, 0));
    auto physics_ball = PhysicsBall(world, initial_ball_state, 1.0, 9.8);

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
    physics_ball.chip(Vector(2, 0));
    EXPECT_TRUE(physics_ball.isInFlight());

    for (unsigned int i = 0; i < 100; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);

        // The ball should be in flight until it has passed the obstacle box
        auto ball = physics_ball.getBallState();
        // Polygons have a "skin" and some slop in Box2D so the ball may move a bit
        // further than expected before the collision stops and isInFlight() returns
        // false, but because chipping isn't perfect in the real world we only need this
        // to be close enough
        if (ball.position().x() - BALL_MAX_RADIUS_METERS <= 2.53)
        {
            EXPECT_TRUE(physics_ball.isInFlight());
        }
        else
        {
            EXPECT_FALSE(physics_ball.isInFlight());
        }
    }

    auto ball = physics_ball.getBallState();
    EXPECT_GT(ball.position().x() - BALL_MAX_RADIUS_METERS, 2.53);
    EXPECT_FALSE(physics_ball.isInFlight());
}
