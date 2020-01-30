#pragma once

#include "software/world/world.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/backend/simulation/simulator_robot.h"
#include "software/ai/primitive/primitive.h"

class Simulator {
public:
    explicit Simulator(const World& world);
    Simulator() = delete;

    void stepSimulation(const Duration time_step);
    void setPrimitives(ConstPrimitiveVectorPtr primitives);
    World getWorld();

private:
    PhysicsWorld physics_world;
    std::vector<SimulatorRobot> friendly_simulator_robots;
};

