#pragma once

#include "software/ai/primitive/primitive.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/simulator_robot.h"
#include "software/world/world.h"

class Simulator
{
   public:
    explicit Simulator(const World& world);
    Simulator() = delete;

    void stepSimulation(const Duration& time_step);
    void setPrimitives(ConstPrimitiveVectorPtr primitives);
    World getWorld();

   private:
    PhysicsWorld physics_world;
    std::vector<std::shared_ptr<SimulatorRobot>> friendly_simulator_robots;
    std::shared_ptr<SimulatorBall> simulator_ball;
};
