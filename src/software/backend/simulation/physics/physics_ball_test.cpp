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
    auto physics_ball = PhysicsBall(world, ball_parameter);
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

    auto physics_ball = PhysicsBall(world, ball_parameter);

    EXPECT_EQ(1, world->GetBodyCount());
}

TEST(PhysicsBallTest, test_physics_ball_is_removed_from_world_when_destroyed)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    {
        Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_ball = PhysicsBall(world, ball_parameter);

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
    auto physics_ball = PhysicsBall(world, ball_parameter);

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
    auto physics_ball = PhysicsBall(world, ball_parameter);

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
    auto physics_ball = PhysicsBall(world, ball_parameter);

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
    auto physics_ball = PhysicsBall(world, ball_parameter);

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
