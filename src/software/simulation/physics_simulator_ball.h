#pragma once

#include <functional>
#include <memory>

#include "software/simulation/physics/physics_ball.h"
#include "software/simulation/simulator_ball.h"

/**
 * The SimulatorBall class acts as a wrapper for a PhysicsBall that deals with more
 * logic-focused elements for simulation.
 */
class PhysicsSimulatorBall : public SimulatorBall
{
  public:
    /**
     * Create a new PhysicsSimulatorBall given a PhysicsBall
     *
     * @param physics_ball the PhysicsBall to simulate and control
     */
    explicit PhysicsSimulatorBall(std::weak_ptr<PhysicsBall> physics_ball);
    explicit PhysicsSimulatorBall() = delete;

    /**
     * Returns the current position of the ball, in global field coordinates, in meters
     *
     * @return the current position of the ball, in global field coordinates, in meters
     */
    Point position() const override;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    Vector velocity() const override;

   private:
    /**
     * Helper functions that check if the current pointer to the physics_ball is valid
     * before calling the given function. If the physics_ball is invalid, a warning is
     * logged and a default value is returned;
     *
     * @param func The function to perform on the physics ball
     */
    Point checkValidAndReturnPoint(
        std::function<Point(const std::shared_ptr<PhysicsBall>)> func) const;
    Vector checkValidAndReturnVector(
        std::function<Vector(const std::shared_ptr<PhysicsBall>)> func) const;

    std::weak_ptr<PhysicsBall> physics_ball
}