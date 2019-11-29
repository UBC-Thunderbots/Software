#include "software/backend/simulation/physics/physics_field.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <math.h>

#include "software/backend/simulation/physics/physics_ball.h"
#include "software/test_util/test_util.h"
#include "software/world/field.h"

TEST(PhysicsFieldTest, test_get_field_with_timestamp)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());
    auto physics_field = PhysicsField(world, field_parameter);
    auto field         = physics_field.getFieldWithTimestamp(Timestamp::fromSeconds(3.3));

    EXPECT_EQ(field_parameter, field);
    EXPECT_EQ(Timestamp::fromSeconds(3.3), field.getMostRecentTimestamp());
}

TEST(PhysicsFieldTest, test_field_added_to_physics_world_on_creation)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());

    EXPECT_EQ(0, world->GetBodyCount());

    auto physics_field = PhysicsField(world, field_parameter);

    // The field adds 3 bodies: 1 for the boundary and 1 for each goal
    EXPECT_EQ(3, world->GetBodyCount());
}

TEST(PhysicsFieldTest, test_physics_field_is_removed_from_world_when_destroyed)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    {
        Field field_parameter(::Test::TestUtil::createSSLDivBField());

        EXPECT_EQ(0, world->GetBodyCount());

        auto physics_field = PhysicsField(world, field_parameter);

        EXPECT_EQ(3, world->GetBodyCount());
    }

    // Once we leave the above scope the field is destroyed, so it should have been
    // removed from the world
    EXPECT_EQ(0, world->GetBodyCount());
}

TEST(PhysicsFieldTest, test_field_dimensions_do_not_change_during_simulation_step)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());
    auto physics_field = PhysicsField(world, field_parameter);

    // We have to take lots of small steps because a significant amount of accuracy
    // is lost if we take a single step of 1 second
    for (unsigned int i = 0; i < 60; i++)
    {
        // 5 and 8 here are somewhat arbitrary values for the velocity and position
        // iterations but are the recommended defaults from
        // https://www.iforce2d.net/b2dtut/worlds
        world->Step(1.0 / 60.0, 5, 8);
    }

    auto field = physics_field.getFieldWithTimestamp(Timestamp::fromSeconds(3.3));
    EXPECT_EQ(field, field_parameter);
}

TEST(PhysicsBallTest, test_ball_bounces_off_field_boundary)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());
    auto physics_field = PhysicsField(world, field_parameter);

    Ball ball_parameter(field_parameter.friendlyHalf().posXPosYCorner(), Vector(0, 2),
                        Timestamp::fromSeconds(0));
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
    EXPECT_LT((Vector(0, -2) - ball.velocity()).length(), 1e-5);
}

TEST(PhysicsBallTest, test_ball_bounces_off_enemy_goal)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());
    auto physics_field = PhysicsField(world, field_parameter);

    Ball ball_parameter(field_parameter.enemyGoal() + Vector(-1, 0), Vector(3.0, 0),
                        Timestamp::fromSeconds(0));
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
    EXPECT_LT((Vector(-3.0, 0.0) - ball.velocity()).length(), 1e-3);
}

TEST(PhysicsBallTest, test_ball_bounces_off_friendly_goal)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    Field field_parameter(::Test::TestUtil::createSSLDivBField());
    auto physics_field = PhysicsField(world, field_parameter);

    Ball ball_parameter(field_parameter.friendlyGoal() + Vector(1, 0), Vector(-3.0, 0),
                        Timestamp::fromSeconds(0));
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
    EXPECT_LT((Vector(3.0, 0.0) - ball.velocity()).length(), 1e-3);
}
