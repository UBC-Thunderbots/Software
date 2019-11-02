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

    /**
     * Removes the corresponding field bodies from the Box2D world. If this function
     * is called on a world that does not contain the field, nothing happens.
     *
     * @param world The Box2D world to remove the ball from
     */
    void removeFromWorld(std::shared_ptr<b2World> world);

    /**
     * Returns a Field object representing the current state of the field object in the
     * simulated Box2D world the field was created in. The timestamp is provided as a
     * parameter so that the caller can control the timestamp of the data being returned,
     * since the caller will have context about the Box2D world and simulation time step,
     * and can synchronize the Field timestamp with other objects.
     *
     * @param timestamp The timestamp for the returned Field to have
     *
     * @return A Field object representing the current state of the field object in the
     * simulated Box2D world the field was originally created in. The returned Field
     * object will have the same timestamp as the one provided in the parameter
     */
    Field getFieldWithTimestamp(const Timestamp& timestamp) const;

   private:
    void setupFieldBoundary(std::shared_ptr<b2World> world, const Field& field);
    void setupEnemyGoal(std::shared_ptr<b2World> world, const Field& field);
    void setupFriendlyGoal(std::shared_ptr<b2World> world, const Field& field);

    // This body represents the 4 boundary walls around the edge of the field
    b2BodyDef field_boundary_body_def;
    b2ChainShape field_boundary_shape;
    b2Body* field_boundary_body;
    b2FixtureDef field_boundary_fixture_def;

    // This body represents the enemy net
    b2BodyDef enemy_goal_body_def;
    b2ChainShape enemy_goal_shape;
    b2Body* enemy_goal_body;
    b2FixtureDef enemy_goal_fixture_def;

    // This body represents the friendly goal
    b2BodyDef friendly_goal_body_def;
    b2ChainShape friendly_goal_shape;
    b2Body* friendly_goal_body;
    b2FixtureDef friendly_goal_fixture_def;

    Field field;
};
