#include "software/simulation/physics/box2d_util.h"

#include <Box2D/Box2D.h>
#include <gtest/gtest.h>

#include "software/new_geom/point.h"

TEST(Box2DUtilTest, test_existence_of_null_body_in_null_world)
{
    b2World* world = nullptr;

    b2Body* body = nullptr;

    auto result = bodyExistsInWorld(body, world);
    EXPECT_FALSE(result);
}

TEST(Box2DUtilTest, test_existence_of_null_body)
{
    b2Vec2 gravity(0, 0);
    auto world = new b2World(gravity);

    b2Body* body = nullptr;

    auto result = bodyExistsInWorld(body, world);
    EXPECT_FALSE(result);
}

TEST(Box2DUtilTest, test_existence_of_random_body_in_empty_world)
{
    b2Vec2 gravity(0, 0);
    auto world       = new b2World(gravity);
    auto other_world = new b2World(gravity);

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
    auto world = new b2World(gravity);

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
    auto world       = new b2World(gravity);
    auto other_world = new b2World(gravity);

    b2BodyDef body_def_1;
    body_def_1.type = b2_dynamicBody;
    body_def_1.position.Set(0, 0);
    body_def_1.linearVelocity.Set(0, 0);
    auto body_1 = world->CreateBody(&body_def_1);

    b2BodyDef body_def_2;
    body_def_2.type = b2_dynamicBody;
    body_def_2.position.Set(1, 0.8f);
    body_def_2.linearVelocity.Set(0, 0);
    auto body_2 = world->CreateBody(&body_def_2);

    b2BodyDef body_def_3;
    body_def_3.type = b2_staticBody;
    body_def_3.position.Set(1, 0.8f);
    auto body_3 = world->CreateBody(&body_def_3);

    b2BodyDef body_def_4;
    body_def_4.type = b2_dynamicBody;
    body_def_4.position.Set(1, 0.8f);
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
    b2Vec2 expected = {-1.02f, 5.4f};
    EXPECT_EQ(expected, createVec2(Point(-1.02, 5.4)));

    expected = {0.0f, 100.100f};
    EXPECT_EQ(expected, createVec2(Point(0.0, 100.100)));
}

TEST(Box2DUtilTest, test_create_b2Vec2_from_vector)
{
    b2Vec2 expected = {-1.02f, 5.4f};
    EXPECT_EQ(expected, createVec2(Vector(-1.02f, 5.4f)));

    expected = {0.0f, 100.100f};
    EXPECT_EQ(expected, createVec2(Vector(0.0, 100.100)));
}

TEST(Box2DUtilTest, test_create_point_from_b2Vec2)
{
    Point expected(-1.02, 5.4);
    EXPECT_LT((expected - createPoint(b2Vec2({-1.02f, 5.4f}))).length(), 0.001);

    expected = Point(0.0, 100.100);
    EXPECT_LT((expected - createPoint(b2Vec2({0.0f, 100.100f}))).length(), 0.001);
}

TEST(Box2DUtilTest, test_create_vector_from_b2Vec2)
{
    Vector expected(-1.02, 5.4);
    EXPECT_LT((expected - createVector(b2Vec2({-1.02f, 5.4f}))).length(), 0.001);

    expected = Vector(0.0, 100.100);
    EXPECT_LT((expected - createVector(b2Vec2({0.0f, 100.100f}))).length(), 0.001);
}

TEST(Box2DUtilTest, test_polygon_area_with_triangle)
{
    const unsigned int num_vertices       = 3;
    b2Vec2 polygon_vertices[num_vertices] = {{0, 0}, {1, 0}, {0, 1}};
    b2PolygonShape polygon;
    polygon.Set(polygon_vertices, num_vertices);

    float result = polygonArea(polygon);
    EXPECT_FLOAT_EQ(result, 0.5);
}

TEST(Box2DUtilTest, test_polygon_area_with_rectangle)
{
    const unsigned int num_vertices       = 4;
    b2Vec2 polygon_vertices[num_vertices] = {{-1, -1}, {-1, 3}, {3.5, 3}, {3.5, -1}};
    b2PolygonShape polygon;
    polygon.Set(polygon_vertices, num_vertices);

    float result = polygonArea(polygon);
    EXPECT_FLOAT_EQ(result, 18.0);
}

TEST(Box2DUtilTest, test_polygon_area_with_irregular_polygon)
{
    const unsigned int num_vertices       = 5;
    b2Vec2 polygon_vertices[num_vertices] = {
        {-3, -2}, {-1, -4}, {6, 1}, {3, 10}, {-4, 9},
    };
    b2PolygonShape polygon;
    polygon.Set(polygon_vertices, num_vertices);

    float result = polygonArea(polygon);
    EXPECT_FLOAT_EQ(result, 96.0);
}
