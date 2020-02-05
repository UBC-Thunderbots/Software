#include "software/backend/simulation/physics/physics_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "software/world/ball.h"

TEST(PhysicsBallTest, test_get_ball_with_timestamp)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);
    auto ball         = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_TRUE(ball_parameter.position().isClose(ball.position(), 1e-7));
    EXPECT_LT((ball_parameter.velocity() - ball.velocity()).length(), 1e-7);
    EXPECT_EQ(Timestamp::fromSeconds(1.1), ball.lastUpdateTimestamp());
}

TEST(PhysicsBallTest, test_ball_added_to_physics_world_on_creation)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST(PhysicsBallTest, test_physics_ball_is_removed_from_world_when_destroyed)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    {
        Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

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

    Ball ball_parameter(Point(1, -1), Vector(1, -2), Timestamp::fromSeconds(0));
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

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_TRUE(Point(2, -3).isClose(ball.position(), 0.01));
    EXPECT_LT((Vector(1, -2) - ball.velocity()).length(), 1e-5);
    EXPECT_EQ(Timestamp::fromSeconds(1.1), ball.lastUpdateTimestamp());
}

TEST(PhysicsBallTest, test_ball_acceleration_and_velocity_updates_during_simulation_step)
{
    b2Vec2 gravity(3, -0.5);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
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

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_LT((Vector(3, -0.5) - ball.velocity()).length(), 0.01);
    EXPECT_EQ(Timestamp::fromSeconds(1.1), ball.lastUpdateTimestamp());
}

TEST(PhysicsBallTest, test_ball_changes_reverses_direction_after_object_collision)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

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

    Ball ball_parameter(Point(0, 0), Vector(0.5, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 120; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_LT((Vector(-0.5, 0.0) - ball.velocity()).length(), 1e-7);
}

TEST(PhysicsBallTest, test_ball_changes_changes_direction_after_object_deflection)
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

    Ball ball_parameter(Point(0, 0), Vector(1.0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 120; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_LT((Vector(0.0, -1.0) - ball.velocity()).length(), 1e-5);
}

TEST(PhysicsBallTest, test_apply_force_to_ball) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    // Apply force for 1 second
    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for(unsigned int i = 0; i < 60; i++) {
        physics_ball.applyForce(Vector(1, 2));
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST(PhysicsBallTest, test_apply_impulse_to_ball) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    physics_ball.applyImpulse(Vector(1, 2));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);
    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT((ball.velocity() - Vector(1, 2)).length(), 0.05);
}

TEST(PhysicsBallTest, test_kick_ball) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    physics_ball.kick(Vector(-2, 3));
    // 5 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    world->Step(1.0 / 60.0, 5, 8);
    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_LT((ball.velocity() - Vector(-2, 3)).length(), 0.05);
}

TEST(PhysicsBallTest, test_chip_ball_without_collisions) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.chip(Vector(1, 0));
    EXPECT_TRUE(physics_ball.isInFlight());

    for(unsigned int i = 0; i < 120; i++) {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
        auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
        if(ball.position().x() < 1.0) {
            EXPECT_TRUE(physics_ball.isInFlight());
        }else {
            EXPECT_FALSE(physics_ball.isInFlight());
        }
    }

    EXPECT_FALSE(physics_ball.isInFlight());
}

TEST(PhysicsBallTest, test_chip_ball_with_collisions) {
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);
    Ball ball_parameter(Point(0, 0), Vector(0, 0), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter, 1.0, 9.8);

    EXPECT_FALSE(physics_ball.isInFlight());
    physics_ball.chip(Vector(2, 0));
    EXPECT_TRUE(physics_ball.isInFlight());

    for(unsigned int i = 0; i < 40; i++) {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
        EXPECT_TRUE(physics_ball.isInFlight());
    }

    physics_ball.incrementNumCurrentCollisions();

    for(unsigned int i = 0; i < 50; i++) {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
        EXPECT_TRUE(physics_ball.isInFlight());
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(0));
    EXPECT_GT(ball.position().x(), 2.0);

    physics_ball.decrementNumCurrentCollisions();
    EXPECT_FALSE(physics_ball.isInFlight());
}
