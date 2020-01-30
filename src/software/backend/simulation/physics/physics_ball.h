#pragma once

#include <Box2D/Box2D.h>

#include "software/new_geom/point.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"

// Forward-delcare to avoid circular dependencies
class SimulatorBall;

/**
 * This class represents a ball in a Box2D physics simulation. It provides a convenient
 * way for us to abstract the ball and convert to our own Ball class when data is needed
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
     * @param ball The Ball to be created in the Box2D world
     * @param mass_kg The mass of the ball in kg
     */
    explicit PhysicsBall(std::shared_ptr<b2World> world, const Ball& ball, const double mass_kg);

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
     * Returns a Ball object representing the current state of the ball object in the
     * simulated Box2D world the ball was created in. The timestamp is provided as a
     * parameter so that the caller can control the timestamp of the data being returned,
     * since the caller will have context about the Box2D world and simulation time step,
     * and can synchronize the Ball timestamp with other objects.
     *
     * @param timestamp The timestamp for the returned Ball to have
     *
     * @return A Ball object representing the current state of the ball object in the
     * simulated Box2D world the ball was originally created in. The returned Ball object
     * will have the same timestamp as the one provided in the parameter
     */
    Ball getBallWithTimestamp(const Timestamp& timestamp) const;

    /**
     * Returns the mass of the ball in kg
     *
     * @return the mass of the ball in kg
     */
    double getMassKg() const;

    /**
     * Applies the given force vector to the ball at its center of mass
     *
     * @param force The force to apply
     */
    void applyForce(const Vector& force);

    /**
     * Applies the given impulse vector to the ball at its center of mass
     *
     * @param impulse The impulse to apply
     */
    void applyImpulse(const Vector& impulse);

    // TODO: test
    /**
     * Sets the SimulatorBall that this PhysicsBall is associated with
     *
     * @param simulator_ball A pointer to the SimulatorBall that this PhysicsBall
     * is associated with
     */
    void setSimulatorBall(SimulatorBall* simulator_ball);

    /**
     * Returns a pointer to the SimulatorBall this PhysicsBall is associated with
     *
     * @return a pointer to the SimulatorBall this PhysicsBall is associated with
     */
    SimulatorBall* getSimulatorBall() const;

   private:
    // See https://box2d.org/manual.pdf chapters 6 and 7 more information on Shapes,
    // Bodies, and Fixtures
    b2CircleShape ball_shape;
    b2BodyDef ball_body_def;
    b2FixtureDef ball_fixture_def;
    b2Body* ball_body;

    // The SimulatorBall associated with this PhysicsBall, if any
    SimulatorBall* simulator_ball;
};
