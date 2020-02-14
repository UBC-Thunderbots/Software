#pragma once

#include <memory>

#include "software/backend/simulation/physics/physics_ball.h"

/**
 * The SimulatorBall class acts as a wrapper for a PhysicsBall that deals with more
 * logic-focused elements for simulation.
 */
class SimulatorBall
{
   public:
    /**
     * Create a new SimulatorBall given a PhysicsBall
     *
     * @param physics_ball the PhysicsBall to simulate and control
     */
    explicit SimulatorBall(std::weak_ptr<PhysicsBall> physics_ball);
    explicit SimulatorBall() = delete;

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

   private:
    std::weak_ptr<PhysicsBall> physics_ball;
};
