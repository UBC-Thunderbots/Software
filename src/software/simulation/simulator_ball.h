#pragma once

#include "software/geom/point.h"
#include "software/geom/vector.h"

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
    virtual Point position() const = 0;

    /**
     * Returns the current velocity of the ball, in global field coordinates, in m/s
     *
     * @return the current velocity of the ball, in global field coordinates, in m/s
     */
    virtual Vector velocity() const = 0;
};
