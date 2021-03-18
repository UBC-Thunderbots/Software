#pragma once

#include <functional>
#include <memory>

#include "software/simulation/physics/physics_ball.h"

/**
 * The SimulatorBall is an abstract class for simulator ball implementations
 */
class SimulatorBall
{
   public:
    /**
     * Returns the current position of the ball, in global field coordinates, in meters
     *
     * @return the current position of the ball, in global field coordinates, in meters
     */
    Point position() const = 0;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    Vector velocity() const = 0;
};
