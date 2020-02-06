#pragma once

#include <Box2D/Box2D.h>
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

class SimulationContactListener : public b2ContactListener {
public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
    void EndContact(b2Contact* contact) override;

private:
    // TODO: comment
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallChickerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallDribblerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallRobotBodyContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    PhysicsBall* isBallContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
};
