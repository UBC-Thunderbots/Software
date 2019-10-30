#include "software/backend/simulation/physics_field.h"

#include "software/backend/simulation/box2d_util.h"

PhysicsField::PhysicsField(std::shared_ptr<b2World> world, const Field &field)
    : field(field)
{
    setupFieldBoundary(world, field);
    setupEnemyGoal(world, field);
    setupFriendlyGoal(world, field);
}

void PhysicsField::setupFieldBoundary(std::shared_ptr<b2World> world, const Field &field)
{
    field_boundary_body_def.type = b2_staticBody;
    // Note that the body shape is defined relative to the body position. Setting the
    // body position to (0, 0) makes it easy to add the shape using the standard field
    // interface, since field coordinates are also relative to (0, 0)
    // https://www.iforce2d.net/b2dtut/fixtures
    field_boundary_body_def.position.Set(0, 0);
    field_boundary_body = world->CreateBody(&field_boundary_body_def);

    const unsigned int num_field_boundary_vertices              = 4;
    b2Vec2 field_boundary_vertices[num_field_boundary_vertices] = {
        createVec2(field.fieldBoundary().posXPosYCorner()),
        createVec2(field.fieldBoundary().posXNegYCorner()),
        createVec2(field.fieldBoundary().negXNegYCorner()),
        createVec2(field.fieldBoundary().negXPosYCorner()),
    };
    field_boundary_shape.CreateLoop(field_boundary_vertices, num_field_boundary_vertices);
    field_boundary_fixture_def.shape = &field_boundary_shape;
    // Collisions with the field boundary are perfectly elastic and have
    // no friction
    field_boundary_fixture_def.restitution = 1.0;
    field_boundary_fixture_def.friction    = 0.0;
    field_boundary_body->CreateFixture(&field_boundary_fixture_def);
}

void PhysicsField::setupEnemyGoal(std::shared_ptr<b2World> world, const Field &field)
{
    enemy_goal_body_def.type = b2_staticBody;
    // Note that the body shape is defined relative to the body position. Setting the
    // body position to (0, 0) makes it easy to add the shape using the standard field
    // interface, since field coordinates are also relative to (0, 0)
    // https://www.iforce2d.net/b2dtut/fixtures
    enemy_goal_body_def.position.Set(0, 0);
    enemy_goal_body = world->CreateBody(&enemy_goal_body_def);

    const unsigned int num_enemy_goal_vertices          = 4;
    b2Vec2 enemy_goal_vertices[num_enemy_goal_vertices] = {
        createVec2(field.enemyGoalpostNeg()),
        createVec2(field.enemyGoalpostNeg() + Vector(field.goalXLength(), 0)),
        createVec2(field.enemyGoalpostPos() + Vector(field.goalXLength(), 0)),
        createVec2(field.enemyGoalpostPos())};
    enemy_goal_shape.CreateChain(enemy_goal_vertices, num_enemy_goal_vertices);
    enemy_goal_fixture_def.shape = &enemy_goal_shape;
    // Collisions with the enemy goal are perfectly elastic and have no friction
    enemy_goal_fixture_def.restitution = 1.0;
    enemy_goal_fixture_def.friction    = 1.0;
    enemy_goal_body->CreateFixture(&enemy_goal_fixture_def);
}

void PhysicsField::setupFriendlyGoal(std::shared_ptr<b2World> world, const Field &field)
{
    friendly_goal_body_def.type = b2_staticBody;
    // Note that the body shape is defined relative to the body position. Setting the
    // body position to (0, 0) makes it easy to add the shape using the standard field
    // interface, since field coordinates are also relative to (0, 0)
    // https://www.iforce2d.net/b2dtut/fixtures
    friendly_goal_body_def.position.Set(0, 0);
    friendly_goal_body = world->CreateBody(&friendly_goal_body_def);

    const unsigned int num_friendly_goal_vertices             = 4;
    b2Vec2 friendly_goal_vertices[num_friendly_goal_vertices] = {
        createVec2(field.friendlyGoalpostNeg()),
        createVec2(field.friendlyGoalpostNeg() - Vector(field.goalXLength(), 0)),
        createVec2(field.friendlyGoalpostPos() - Vector(field.goalXLength(), 0)),
        createVec2(field.friendlyGoalpostPos())};
    friendly_goal_shape.CreateChain(friendly_goal_vertices, num_friendly_goal_vertices);
    friendly_goal_fixture_def.shape = &friendly_goal_shape;
    // Collisions with the friendly goal are perfectly elastic and have no friction
    friendly_goal_fixture_def.restitution = 1.0;
    friendly_goal_fixture_def.friction    = 1.0;
    friendly_goal_body->CreateFixture(&friendly_goal_fixture_def);
}

void PhysicsField::removeFromWorld(std::shared_ptr<b2World> world)
{
    if (bodyExistsInWorld(field_boundary_body, world))
    {
        world->DestroyBody(field_boundary_body);
    }

    if (bodyExistsInWorld(enemy_goal_body, world))
    {
        world->DestroyBody(enemy_goal_body);
    }

    if (bodyExistsInWorld(friendly_goal_body, world))
    {
        world->DestroyBody(friendly_goal_body);
    }
}

Field PhysicsField::getFieldWithTimestamp(const Timestamp &timestamp) const
{
    Field ret = field;
    ret.updateTimestamp(timestamp);
    return ret;
}
