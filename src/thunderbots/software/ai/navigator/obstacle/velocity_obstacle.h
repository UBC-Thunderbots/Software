#pragma once

#include "ai/navigator/obstacle/obstacle.h"
#include "shared/constants.h"

// Placeholder obstacle represents a stationary robot centred at (0,0)
class VelocityObstacle : public Obstacle
{
   public:
    explicit VelocityObstacle();

    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer scaled by radiusScaling and
     * velocity_projection_scaling
     *
     * @param robot robot to create obstacle boundary polygon around
     * @param radiusScaling safety factor to vary the radius size of the buffer zone
     * (1=default), must be greater than 0
     * @param velocity_projection_scaling scales the projection buffer in the direction of
     * the velocity (1=default), must be greater than 0
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Polygon getBoundaryPolygon(const Robot& robot, double robot_radius_scaling,
                                      double velocity_projection_scaling);
};
