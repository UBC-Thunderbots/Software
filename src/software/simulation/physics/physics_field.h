#pragma once

#include "Box2D/Box2D.h"
#include "software/world/field.h"

/**
 * This class represents a Field in a Box2D physics simulation. It provides a convenient
 * way for us to abstract the field and convert to our own Field class when data is
 * needed.
 *
 * In a PhysicsField, objects are able to collide with the field border, and the nets at
 * each end of the field
 */
class PhysicsField
{
   public:
    /**
     * Creates a new PhysicsField given a Box2D world and a Field object. Several Box2D
     * bodies representing the field will be automatically added to the Box2D world and
     * updated during world update steps.
     *
     * @param world A shared_ptr to a Box2D World
     * @param field The Field to be created in the Box2D world
     */
    explicit PhysicsField(std::shared_ptr<b2World> world, const Field& field);

    PhysicsField() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsField& operator=(const PhysicsField&) = delete;
    PhysicsField(const PhysicsField&)            = delete;

    /**
     * Destroys the PhysicsField object and removes any corresponding bodies from
     * the physics world if the field is part of one
     */
    ~PhysicsField();

    /**
     * Returns the current state of the Field
     *
     * @return the current state of the Field
     */
    Field getField() const;

   private:
    /**
     * A helper function that creates a physics body to represent the Field
     * in the physics world
     *
     * @param world The physics world to setup the Field in
     */
    void createFieldBody(std::shared_ptr<b2World> world);

    /**
     * A helper function that creates and sets up physics objects for the field
     * boundary in the physics world
     *
     * @param field The Field being created in the physics world
     */
    void setupFieldBoundary(const Field& field);

    /**
     * A helper function that creates and sets up physics objects for the enemy
     * goal in the physics world
     *
     * @param field The Field being created in the physics world
     */
    void setupEnemyGoal(const Field& field);

    /**
     * A helper function that creates and sets up physics objects for the friendly
     * goal in the physics world
     *
     * @param field The Field being created in the physics world
     */
    void setupFriendlyGoal(const Field& field);

    // This body represents the entire field object. Different fixtures and shapes
    // are added to this body for the field boundary, goals, etc. The fixtures are what
    // actually handle collisions, which is why we only need a single body to represent
    // the whole field
    b2BodyDef field_body_def;
    b2Body* field_body;

    b2ChainShape field_boundary_shape;
    b2FixtureDef field_boundary_fixture_def;

    b2ChainShape enemy_goal_shape;
    b2FixtureDef enemy_goal_fixture_def;

    b2ChainShape friendly_goal_shape;
    b2FixtureDef friendly_goal_fixture_def;

    // These values are somewhat arbitrary since the restitution and friction of the ball
    // also affects any collision behavior. We implement custom collision logic in the
    // contact listener to handle ball-field collisions
    static constexpr float FIELD_WALL_RESTITUTION = 0.0f;
    static constexpr float FIELD_WALL_FRICTION    = 0.0f;

    // Since the field never changes during simulation, we store the initial field
    // used during construction to make it easy to return Field objects when requested
    Field field;
};
