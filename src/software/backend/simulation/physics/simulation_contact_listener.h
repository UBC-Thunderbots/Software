#pragma once

#include <Box2D/Box2D.h>
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"
#include "software/backend/simulation/physics/physics_object_user_data.h"

class SimulationContactListener : public b2ContactListener {
public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
    void EndContact(b2Contact* contact) override;

private:
    std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> isBallChickerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> isBallDribblerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    void handleBallChickerContact(b2Contact* contact, SimulatorBall* ball, SimulatorRobot* robot);
    void handleBallDribblerContact(b2Contact* contact, SimulatorBall* ball, SimulatorRobot* robot);
};
