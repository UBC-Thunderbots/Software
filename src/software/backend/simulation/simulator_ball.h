#pragma once

#include "software/backend/simulation/physics/physics_ball.h"
#include <memory>

// TODO: comment
class SimulatorBall {
public:
    explicit SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball);
    explicit SimulatorBall() = default;

    /**
     * Returns the x-position of the ball, in global field coordinates, in meters
     *
     * @return the x-position of the ball, in global field coordinates, in meters
     */
    float getPositionX() const;

    /**
     * Returns the y-position of the ball, in global field coordinates, in meters
     *
     * @return the y-position of the ball, in global field coordinates, in meters
     */
    float getPositionY() const;

    /**
     * Returns the current position of the ball, in global field coordinates, in meters
     *
     * @return the current position of the ball, in global field coordinates, in meters
     */
    Point position() const;

    /**
     * Returns the x-velocity of the ball, in global field coordinates, in m/s
     *
     * @return the x-velocity of the ball, in global field coordinates, in m/s
     */
    float getVelocityX() const;

    /**
     * Returns the y-velocity of the ball, in global field coordinates, in m/s
     *
     * @return the y-velocity of the ball, in global field coordinates, in m/s
     */
    float getVelocityY() const;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    Vector velocity() const;

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

private:
    void onBallContact(PhysicsBall* ball);

    std::weak_ptr<PhysicsBall> physics_ball;
};