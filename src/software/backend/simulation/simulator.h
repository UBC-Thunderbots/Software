#pragma once

#include "software/ai/primitive/primitive.h"
#include "software/backend/simulation/physics/physics_world.h"
#include "software/backend/simulation/simulator_ball.h"
#include "software/backend/simulation/simulator_robot.h"
#include "software/world/world.h"

/**
 * The Simulator abstracts away the physics simulation of all objects in the world,
 * as well as the firmware simulation for the robots. This provides a simple interface
 * to setup, run, and query the current state of the simulation.
 */
class Simulator
{
   public:
    /**
     * Creates a new Simulator with the given world. The starting state of the simulation
     * will match the state of the given world
     *
     * @param world The world to initialize the simulation with
     */
    explicit Simulator(const World& world);
    Simulator() = delete;

    /**
     * Advances the simulation by the given time step. This will simulate
     * physics and primitives
     *
     * @param time_step how much to advance the simulation by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Sets the primitives being simulated by the robots in simulation
     *
     * @param primitives The primitives to simulate
     */
    void setPrimitives(ConstPrimitiveVectorPtr primitives);

    /**
     * Returns the current state of the world in the simulation
     *
     * @return the current state of the world in the simulation
     */
    World getWorld();

   private:
    PhysicsWorld physics_world;
    std::vector<std::shared_ptr<SimulatorRobot>> friendly_simulator_robots;
    std::shared_ptr<SimulatorBall> simulator_ball;
};
