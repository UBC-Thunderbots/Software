#pragma once

#include <optional>

#include "software/backend/simulation/physics_ball.h"
#include "software/backend/simulation/physics_field.h"
#include "software/util/time/duration.h"
#include "software/util/time/timestamp.h"
#include "software/world/world.h"

/**
 * The PhysicsSimulator handles the physics simulation of 'World' objects, and abstracts
 * away dealing with the physics engine and our PhysicsObject abstraction
 * in the world.
 *
 * The simulation uses Box2D as the physics engine, and the abstractions can be found
 * in the `simulation` folder. For more information and examples of how to use Box2D
 * see the following:
 * - https://box2d.org/documentation/
 * - https://github.com/libgdx/libgdx/wiki/Box2d
 * - https://www.iforce2d.net/b2dtut/
 */
class PhysicsSimulator
{
   public:
    /**
     * Creates a new PhysicsSimulator
     *
     * @param world The initial state of the world in the simulation
     */
    explicit PhysicsSimulator(const World& world);

    // Delete the copy and assignment operators because this class really shouldn't
    // need them and we don't want to risk doing anything nasty with the internal
    // pointers to the physics world this class uses
    PhysicsSimulator& operator=(const PhysicsSimulator&) = delete;
    PhysicsSimulator(const PhysicsSimulator&)            = delete;

    /**
     * Advances the physics simulation by the given time step, and returns the new state
     * of the world
     *
     * @param time_step How much to advance the simulation by
     *
     * @return The new state of the world after simulating the time step
     */
    World stepSimulation(const Duration& time_step);

   private:
    /**
     * Sets the state of the internal physics world used for simulation
     *
     * @param world The new state of the world to start simulation from
     */
    void setWorld(const World& world);

    // The timestamp of the simulated physics world
    Timestamp physics_world_timestamp;
    std::shared_ptr<b2World> physics_world;
    int velocity_iterations;
    int position_iterations;

    // Our abstractions of objects in the physics world
    std::unique_ptr<PhysicsBall> physics_ball;
    std::unique_ptr<PhysicsField> physics_field;
};
