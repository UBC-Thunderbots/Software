#pragma once

#include <Box2D/Box2D.h>

#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_field.h"
#include "software/simulation/physics/physics_object_user_data.h"
#include "software/simulation/physics/physics_robot.h"

/**
 * This class implements a custom ContactListener for a Box2D world so that we
 * can have custom behaviour when objects collide.
 *
 * See https://www.iforce2d.net/b2dtut/collision-callbacks for more info
 */
class SimulationContactListener : public b2ContactListener
{
   public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
    // Called once at the end of a contact
    void EndContact(b2Contact* contact) override;

    /**
     * Given the user data of 2 objects involved in a contact, returns a pair of
     * pointers to the physics objects involved in the contact if there was a contact
     * point between a ball and dribbler damper. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pair of pointers to the physics objects involved in the constact if there
     * was a contact point between a ball and dribbler damper, and returns std::nullopt
     * otherwise
     */
    static std::optional<std::pair<PhysicsBall*, PhysicsRobot*>>
    isDribblerDamperBallContact(PhysicsObjectUserData* user_data_a,
                                PhysicsObjectUserData* user_data_b);

    /**
     * Given the user data of 2 objects involved in a contact, returns a pair of
     * pointers to the physics objects involved in the contact if there was a contact
     * point between a ball and robot dribbler. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pair of pointers to the physics objects involved in the contact if there
     * was a contact point between a ball and robot dribbler, and returns std::nullopt
     * otherwise
     */
    static std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isDribblerBallContact(
        PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);

    /**
     * Given the user data of 2 objects involved in a contact, returns a pointer
     * to the PhysicsBall involved in the contact if there was a contact point between
     * a ball and any other object. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pointers to the PhysicsBall involved in the contact if there was
     * a contact point between a ball and any other object, and returns std::nullopt
     * otherwise
     */
    static PhysicsBall* isBallContact(PhysicsObjectUserData* user_data_a,
                                      PhysicsObjectUserData* user_data_b);

    /**
     * Given the user data of 2 objects involved in a contact, returns a pair of
     * pointers to the physics objects involved in the contact if there was a contact
     * point between a ball and a field wall. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pair of pointers to the physics objects involved in the constact if there
     * was a contact point between a ball and field wall, and returns std::nullopt
     * otherwise
     */
    static std::optional<std::pair<PhysicsBall*, PhysicsField*>> isBallFieldWallContact(
        PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);

   private:
    // Damping factor for ball collisions with field walls or goals
    static constexpr double BALL_FIELD_WALL_COLLISION_DAMPING = 0.8;
};
