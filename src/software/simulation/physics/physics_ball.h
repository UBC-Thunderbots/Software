#pragma once

#include <Box2D/Box2D.h>

#include <optional>

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"
#include "software/world/ball_state.h"

/**
 * This class represents a ball in a Box2D physics simulation. It provides a convenient
 * way for us to abstract the ball and convert to our own Ball class when data is needed.
 */
class PhysicsBall
{
   public:
    /**
     * Creates a new PhysicsBall given a Box2D world and a Ball object. A Box2D body
     * representing the ball will be automatically added to the Box2D world and updated
     * during world update steps.
     *
     * @param world A shared_ptr to a Box2D World
     * @param ball_state The initial state of the ball
     * @param mass_kg The mass of the ball in kg
     * @param gravity The acceleration due to gravity on the ball, in m/s^2
     */
    explicit PhysicsBall(std::shared_ptr<b2World> world, const BallState& ball_state,
                         const double mass_kg, const double gravity);

    PhysicsBall() = delete;

    // Delete the copy and assignment operators because copying this class causes
    // issues with the b2World and how it tracks bodies in the world, because as objects
    // are copied and destroyed, they will destroy the bodies in the b2World as well
    PhysicsBall& operator=(const PhysicsBall&) = delete;
    PhysicsBall(const PhysicsBall&)            = delete;

    /**
     * Destroys the PhysicsBall object and removes any corresponding bodies from
     * the physics world if the ball is part of one
     */
    ~PhysicsBall();

    /**
     * Returns the current state of the ball

     * @return The current state of the ball
     */
    BallState getBallState() const;

    /**
     * Returns the current position of the ball, in global field coordinates, in meters
     *
     * @return the current position of the ball, in global field coordinates, in meters
     */
    Point position() const;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    Vector velocity() const;

    /**
     * Kicks the ball in the direction of the given vector, and applies a change in
     * velocity equal to the magnitude of the vector.
     *
     * @param kick_vector The kick vector to apply
     */
    void kick(Vector kick_vector);

    /**
     * Chips the ball in the direction of the given vector. The isInFlight() function will
     * return true until the ball has travelled a distance equal to the magnitude of the
     * vector from its current location when this function is called.
     *
     * @param chip_vector The chip_vector to apply
     */
    void chip(const Vector& chip_vector);

    /**
     * Returns true if the ball is currently in flight / a chip is in progress,
     * and false otherwise
     *
     * @return true if the ball is currently in flight / a chip is in progress,
     * and false otherwise
     */
    bool isInFlight();

    /**
     * Applies the given force vector to the ball at its center of mass
     *
     * @param force The force to apply
     */
    void applyForce(const Vector& force);

    /**
     * Applies the given impulse vector to the ball at its center of mass
     *
     * @param force The impulse to apply
     */
    void applyImpulse(const Vector& impulse);

   private:
    /**
     * Returns true if this ball is touching another object in the physics world
     *
     * @return true if the ball is touching another object in the physics world,
     * and false otherwise
     */
    bool isTouchingOtherObject() const;

    // See https://box2d.org/manual.pdf chapters 6 and 7 more information on Shapes,
    // Bodies, and Fixtures
    b2Body* ball_body;

    // The gravity acting on this ball, pulling it towards the ground
    const double gravity;
    // If the ball is currently being chipped, the chip_origin holds the point
    // where the chip was started from
    std::optional<Point> chip_origin;
    // The distance of the most recent chip
    double chip_distance_meters;

    // These restitution and friction values are somewhat arbitrary. Because this is an
    // "ideal" simulation, we can approximate the ball as having perfectly elastic
    // collisions and no friction. Because we also do not generally depend on specific
    // behaviour when the ball collides with something, getting these values to perfectly
    // match reality isn't too important.
    const double ball_restitution = 1.0;
    const double ball_friction    = 0.0;
};
