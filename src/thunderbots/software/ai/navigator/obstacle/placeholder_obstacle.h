#pragma once

#include "../shared/constants.h"
#include "ai/navigator/obstacle/obstacle.h"

// Placeholder obstacle represents a stationary robot centred at (0,0)
class PlaceholderObstacle : public Obstacle
{
   public:
    explicit PlaceholderObstacle();

    /*
     * Gets the boundary polygon around the centre scaled by radiusScaling
     *
     * @param robot *ignored*
     * @param radiusScaling safety factor to vary the radius size of the buffer zone
     * (1=default), must greater than 0
     * @param velocity_projection_scaling *ignored*
     *
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Polygon getBoundaryPolygon(const Robot& robot, double robot_radius_scaling,
                                      double velocity_projection_scaling);
};
