#include "software/backend/simulation/physics_ball.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "software/world/ball.h"

TEST(PhysicsBallTest, test_get_ball_with_timestamp)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));
    auto physics_ball = PhysicsBall(world, ball_parameter);
    auto ball         = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_TRUE(ball_parameter.position().isClose(ball.position(), 1e-7));
    EXPECT_TRUE(ball_parameter.velocity().isClose(ball.velocity(), 1e-7));
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

TEST(PhysicsBallTest, test_remove_physics_ball_from_world)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_ball = PhysicsBall(world, ball_parameter);

    EXPECT_EQ(1, world->GetBodyCount());

    physics_ball.removeFromWorld(world);

    EXPECT_EQ(0, world->GetBodyCount());
}

TEST(PhysicsBallTest, test_remove_physics_ball_from_world_multiple_times)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Ball ball_parameter(Point(0.1, -0.04), Vector(1, -2), Timestamp::fromSeconds(0));

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_ball = PhysicsBall(world, ball_parameter);

    EXPECT_EQ(1, world->GetBodyCount());

    physics_ball.removeFromWorld(world);

    EXPECT_EQ(0, world->GetBodyCount());

    physics_ball.removeFromWorld(world);

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
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_TRUE(Point(2, -3).isClose(ball.position(), 1e-5));
    EXPECT_TRUE(Vector(1, -2).isClose(ball.velocity(), 1e-5));
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
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto ball = physics_ball.getBallWithTimestamp(Timestamp::fromSeconds(1.1));

    EXPECT_TRUE(Vector(3, -0.5).isClose(ball.velocity(), 1e-5));
    EXPECT_EQ(Timestamp::fromSeconds(1.1), ball.lastUpdateTimestamp());
}
