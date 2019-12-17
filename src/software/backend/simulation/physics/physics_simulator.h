#pragma once

#include <optional>

#include "software/backend/simulation/physics/physics_ball.h"
#include "software/backend/simulation/physics/physics_field.h"
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
 *
 * Why do the PhysicsObject classes use raw pointers?
 *
 * Basically, b2Body objects should only be destroyed by the b2World. The b2Body
 * destructor is marked private to force this to happen. See
 * https://github.com/erincatto/Box2D/blob/master/Box2D/Dynamics/b2Body.h and
 * https://github.com/erincatto/Box2D/blob/master/Box2D/Dynamics/b2Body.cpp
 *
 * This means we can't create smart pointers from the b2Body* returned when we create a
 * body like std::shared_ptr<b2Body>(world->CreateBody(&body_def)). This fails because
 * the smart pointer template will try make a call to the private destructor. Ultimately
 * this is why we have to use raw pointers in these cases.
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

    // 3 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    const unsigned int velocity_iterations = 8;
    const unsigned int position_iterations = 3;

    // Our abstractions of objects in the physics world
    std::unique_ptr<PhysicsBall> physics_ball;
    std::unique_ptr<PhysicsField> physics_field;
};
