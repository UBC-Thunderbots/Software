#pragma once

#include <Box2D/Box2D.h>

#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/physics/physics_field.h"
#include "software/simulation/physics/physics_robot.h"
#include "software/simulation/physics/simulation_contact_listener.h"
#include "software/time/duration.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * This class represents a World in a Box2D physics simulation. It provides a convenient
 * way for us to abstract and hold a lot of the world's contents. It's also used to
 * convert to our own World class when needed.
 */
class PhysicsWorld
{
   public:
    /**
     * Creates a new PhysicsWorld given a World.
     *
     * @param world the world to create
     */
    explicit PhysicsWorld(const World& world);

    PhysicsWorld() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;
    PhysicsWorld(const PhysicsWorld&)            = delete;

    /**
     * Returns the current state of the world
     * @return the current state of the world
     */
    World getWorld() const;

    /**
     * Advances the Box2D world physics by the given time step
     *
     * @param time_step how much to advance the world physics by
     */
    void stepSimulation(const Duration& time_step);

    /**
     * Returns the friendly PhysicsRobots currently in the world
     *
     * @return the friendly PhysicsRobots in the world
     */
    std::vector<std::weak_ptr<PhysicsRobot>> getFriendlyPhysicsRobots() const;

    /**
     * Returns the PhysicsBall currently in the world
     *
     * @return the PhysicsBall in the world
     */
    std::weak_ptr<PhysicsBall> getPhysicsBall() const;

   private:
    // Note: we declare the b2World first so it is destroyed last. If it is destroyed
    // before the physics objects, segfaults will occur due to pointers internal to Box2D
    // https://stackoverflow.com/questions/2254263/order-of-member-constructor-and-destructor-calls
    std::shared_ptr<b2World> b2_world;
    // The timestamp of the simulated physics world
    Timestamp current_timestamp;

    /**
     * Sets the state of the internal world used for simulation
     * @param world the new state of the world to start the simulation from
     */
    void initWorld(const World& world);

    // 3 and 8 here are somewhat arbitrary values for the velocity and position
    // iterations but are the recommended defaults from
    // https://www.iforce2d.net/b2dtut/worlds
    const unsigned int velocity_iterations = 8;
    const unsigned int position_iterations = 3;

    const double acceleration_due_to_gravity = 9.8;

    std::unique_ptr<SimulationContactListener> contact_listener;

    // Abstractions of objects in the world
    std::shared_ptr<PhysicsBall> physics_ball;
    std::shared_ptr<PhysicsField> physics_field;
    std::vector<std::shared_ptr<PhysicsRobot>> friendly_physics_robots;
    // Keep track of enemy physics robots as well, because although we don't
    // necessarily control them with our firmware they can still be
    // simulated to move given initial velocities
    std::vector<std::shared_ptr<PhysicsRobot>> enemy_physics_robots;
};
