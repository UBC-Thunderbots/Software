#pragma once

#include <Box2D/Box2D.h>
#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/simulator_robot.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/physics/physics_robot.h"

/**
 * Different types of physics objects that exist in the simulation world
 */
enum PhysicsObjectType {
    ROBOT_CHICKER,
    ROBOT_DRIBBLER,
    BALL
};

/**
 * This struct contains all the user data we link to Box2D physics objects and fixtures.
 * It gives us a way to refer back to our "Physics..." objects when working with Box2D APIs
 *
 * See https://www.iforce2d.net/b2dtut/user-data for more info about Box2D user data
 */
struct PhysicsObjectUserData {
    PhysicsObjectUserData(PhysicsObjectType type, void* object_ptr) : type(type), physics_object(object_ptr) {}
    PhysicsObjectUserData() = delete;

    PhysicsObjectType type;
    void* physics_object;
};

class SimulationContactListener : public b2ContactListener {
public:
    // Called once at the start of a contact
    void BeginContact(b2Contact* contact) override;
    // Continually called at each physics step for the duration of the contact
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;

private:
    std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> isBallChickerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    std::optional<std::pair<SimulatorBall*, SimulatorRobot*>> isBallDribblerContact(PhysicsObjectUserData* user_data_a, PhysicsObjectUserData* user_data_b);
    void handleBallChickerContact(b2Contact* contact, SimulatorBall* ball, SimulatorRobot* robot);
    void handleBallDribblerContact(b2Contact* contact, SimulatorBall* ball, SimulatorRobot* robot);
};
