#include "software/backend/simulation/box2d_util.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "software/geom/point.h"

TEST(Box2DUtilTest, test_existence_of_null_body_in_null_world)
{
    std::shared_ptr<b2World> world;

    b2Body* body = nullptr;

    auto result = bodyExistsInWorld(body, world);
    EXPECT_FALSE(result);
}

TEST(Box2DUtilTest, test_existence_of_null_body)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    b2Body* body = nullptr;

    auto result = bodyExistsInWorld(body, world);
    EXPECT_FALSE(result);
}

TEST(Box2DUtilTest, test_existence_of_random_body_in_empty_world)
{
    b2Vec2 gravity(0, 0);
    auto world       = std::make_shared<b2World>(gravity);
    auto other_world = std::make_shared<b2World>(gravity);

    b2BodyDef body_def;
    body_def.type = b2_dynamicBody;
    body_def.position.Set(0, 0);
    body_def.linearVelocity.Set(0, 0);
    auto body = other_world->CreateBody(&body_def);

    auto result = bodyExistsInWorld(body, world);
    EXPECT_FALSE(result);
}


TEST(Box2DUtilTest, test_existence_of_body_in_world_with_one_body)
{
    b2Vec2 gravity(0, 0);
    auto world = std::make_shared<b2World>(gravity);

    b2BodyDef body_def;
    body_def.type = b2_dynamicBody;
    body_def.position.Set(0, 0);
    body_def.linearVelocity.Set(0, 0);
    auto body = world->CreateBody(&body_def);

    auto result = bodyExistsInWorld(body, world);
    EXPECT_TRUE(result);
}

TEST(Box2DUtilTest, test_existence_of_body_in_world_with_multiple_bodies)
{
    b2Vec2 gravity(0, 0);
    auto world       = std::make_shared<b2World>(gravity);
    auto other_world = std::make_shared<b2World>(gravity);

    b2BodyDef body_def_1;
    body_def_1.type = b2_dynamicBody;
    body_def_1.position.Set(0, 0);
    body_def_1.linearVelocity.Set(0, 0);
    auto body_1 = world->CreateBody(&body_def_1);

    b2BodyDef body_def_2;
    body_def_2.type = b2_dynamicBody;
    body_def_2.position.Set(1, 0.8);
    body_def_2.linearVelocity.Set(0, 0);
    auto body_2 = world->CreateBody(&body_def_2);

    b2BodyDef body_def_3;
    body_def_3.type = b2_staticBody;
    body_def_3.position.Set(1, 0.8);
    auto body_3 = world->CreateBody(&body_def_3);

    b2BodyDef body_def_4;
    body_def_4.type = b2_dynamicBody;
    body_def_4.position.Set(1, 0.8);
    body_def_4.linearVelocity.Set(0, 0);
    auto body_4 = other_world->CreateBody(&body_def_4);

    auto result_1 = bodyExistsInWorld(body_1, world);
    EXPECT_TRUE(result_1);

    auto result_2 = bodyExistsInWorld(body_2, world);
    EXPECT_TRUE(result_2);

    auto result_3 = bodyExistsInWorld(body_3, world);
    EXPECT_TRUE(result_3);

    auto result_4 = bodyExistsInWorld(body_4, world);
    EXPECT_FALSE(result_4);
}

TEST(Box2DUtilTest, test_create_b2Vec2_from_point)
{
    b2Vec2 expected = {-1.02, 5.4};
    EXPECT_EQ(expected, createVec2(Point(-1.02, 5.4)));

    expected = {0.0, 100.100};
    EXPECT_EQ(expected, createVec2(Point(0.0, 100.100)));
}
