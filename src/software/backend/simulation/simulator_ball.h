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
    float getPositionX();

    /**
     * Returns the y-position of the ball, in global field coordinates, in meters
     *
     * @return the y-position of the ball, in global field coordinates, in meters
     */
    float getPositionY();

    /**
     * Returns the x-velocity of the ball, in global field coordinates, in meters
     *
     * @return the x-velocity of the ball, in global field coordinates, in meters
     */
    float getVelocityX();

    /**
     * Returns the y-velocity of the ball, in global field coordinates, in meters
     *
     * @return the y-velocity of the ball, in global field coordinates, in meters
     */
    float getVelocityY();

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
    std::weak_ptr<PhysicsBall> physics_ball;
};