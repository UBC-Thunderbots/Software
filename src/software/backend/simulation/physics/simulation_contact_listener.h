#pragma once

#include <Box2D/Box2D.h>
#include "software/backend/simulation/physics/physics_object.h"
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"

class SimulationContactListener : public b2ContactListener {
public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;

private:
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallChickerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    std::optional<std::pair<PhysicsBall*, PhysicsRobot*>> isBallDribblerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    void handleBallChickerContact(b2Contact* contact, PhysicsBall* ball, PhysicsRobot* robot);
    void handleBallDribblerContact(b2Contact* contact, PhysicsBall* ball, PhysicsRobot* robot);
};
