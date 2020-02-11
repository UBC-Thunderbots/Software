#pragma once

#include <Box2D/Box2D.h>
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

/**
 * This class implements a custom ContactListener for a Box2D world so that we
 * can have custom behavior when objects collid.
 *
 * See https://www.iforce2d.net/b2dtut/collision-callbacks for more info
 */
class SimulationContactListener : public b2ContactListener {
public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
    // Called once at the end of a contact
    void EndContact(b2Contact* contact) override;

private:
    /**
     * Given the user data of 2 objects involved in a collision, returns a pair of pointers
     * to the physics objects involved in the collision if this was a collision between
     * a ball and robot chicker. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pair of pointers to the physics objects involved in the collision if this was
     * a collision between a ball and robot chicker, and returns std::nullopt otherwise
     */
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallChickerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);

    /**
     * Given the user data of 2 objects involved in a collision, returns a pair of pointers
     * to the physics objects involved in the collision if this was a collision between
     * a ball and robot dribbler. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pair of pointers to the physics objects involved in the collision if this was
     * a collision between a ball and robot dribbler, and returns std::nullopt otherwise
     */
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallDribblerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);

    /**
     * Given the user data of 2 objects involved in a collision, returns a pointer
     * to the PhysicsBall involved in the collision if this was a collision between
     * a ball and any other object. Otherwise returns std::nullopt
     *
     * @param user_data_a The user data for the first object in the contact
     * @param user_data_b The user data for the second object in the contact
     *
     * @return A pointers to the PhysicsBall involved in the collision if this was
     * a collision between a ball and any other object, and returns std::nullopt otherwise
     */
    PhysicsBall* isBallContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
};
