#include "software/simulation/physics/physics_field.h"

#include "software/simulation/physics/box2d_util.h"
#include "software/simulation/physics/physics_object_user_data.h"

PhysicsField::PhysicsField(std::shared_ptr<b2World> world, const Field &field)
    : field(field)
{
    // createFieldBody must be called before the setup functions, so that the b2Body is
    // instantiated before fixtures are added to it.
    createFieldBody(world);
    setupFieldBoundary(field);
    setupEnemyGoal(field);
    setupFriendlyGoal(field);
}

PhysicsField::~PhysicsField()
{
    // Examples for removing bodies safely from
    // https://www.iforce2d.net/b2dtut/removing-bodies
    b2World *field_world = field_body->GetWorld();
    if (bodyExistsInWorld(field_body, field_world))
    {
        field_world->DestroyBody(field_body);
    }
}

void PhysicsField::createFieldBody(std::shared_ptr<b2World> world)
{
    // All the BodyDef must be defined before the body is created.
    // Changes made after aren't reflected
    field_body_def.type = b2_staticBody;
    // Note that the body shape is defined relative to the body position. Setting the
    // body position to (0, 0) makes it easy to add the shape using the standard field
    // interface, since field coordinates are also relative to (0, 0)
    // https://www.iforce2d.net/b2dtut/fixtures
    field_body_def.position.Set(0, 0);
    field_body = world->CreateBody(&field_body_def);
}

void PhysicsField::setupFieldBoundary(const Field &field)
{
    const unsigned int num_field_boundary_vertices              = 4;
    b2Vec2 field_boundary_vertices[num_field_boundary_vertices] = {
        createVec2(field.fieldBoundary().posXPosYCorner()),
        createVec2(field.fieldBoundary().posXNegYCorner()),
        createVec2(field.fieldBoundary().negXNegYCorner()),
        createVec2(field.fieldBoundary().negXPosYCorner()),
    };
    field_boundary_shape.CreateLoop(field_boundary_vertices, num_field_boundary_vertices);
    field_boundary_fixture_def.shape       = &field_boundary_shape;
    field_boundary_fixture_def.restitution = FIELD_WALL_RESTITUTION;
    field_boundary_fixture_def.friction    = FIELD_WALL_FRICTION;
    field_boundary_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::FIELD_WALL, this});
    field_body->CreateFixture(&field_boundary_fixture_def);
}

void PhysicsField::setupEnemyGoal(const Field &field)
{
    const unsigned int num_enemy_goal_vertices          = 4;
    b2Vec2 enemy_goal_vertices[num_enemy_goal_vertices] = {
        createVec2(field.enemyGoalpostNeg()),
        createVec2(field.enemyGoalpostNeg() + Vector(field.goalXLength(), 0)),
        createVec2(field.enemyGoalpostPos() + Vector(field.goalXLength(), 0)),
        createVec2(field.enemyGoalpostPos())};
    enemy_goal_shape.CreateChain(enemy_goal_vertices, num_enemy_goal_vertices);
    enemy_goal_fixture_def.shape       = &enemy_goal_shape;
    enemy_goal_fixture_def.restitution = FIELD_WALL_RESTITUTION;
    enemy_goal_fixture_def.friction    = FIELD_WALL_FRICTION;
    enemy_goal_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::FIELD_WALL, this});
    field_body->CreateFixture(&enemy_goal_fixture_def);
}

void PhysicsField::setupFriendlyGoal(const Field &field)
{
    const unsigned int num_friendly_goal_vertices             = 4;
    b2Vec2 friendly_goal_vertices[num_friendly_goal_vertices] = {
        createVec2(field.friendlyGoalpostNeg()),
        createVec2(field.friendlyGoalpostNeg() - Vector(field.goalXLength(), 0)),
        createVec2(field.friendlyGoalpostPos() - Vector(field.goalXLength(), 0)),
        createVec2(field.friendlyGoalpostPos())};
    friendly_goal_shape.CreateChain(friendly_goal_vertices, num_friendly_goal_vertices);
    friendly_goal_fixture_def.shape       = &friendly_goal_shape;
    friendly_goal_fixture_def.restitution = FIELD_WALL_RESTITUTION;
    friendly_goal_fixture_def.friction    = FIELD_WALL_FRICTION;
    friendly_goal_fixture_def.userData =
        new PhysicsObjectUserData({PhysicsObjectType::FIELD_WALL, this});
    field_body->CreateFixture(&friendly_goal_fixture_def);
}

Field PhysicsField::getField() const
{
    return field;
}
