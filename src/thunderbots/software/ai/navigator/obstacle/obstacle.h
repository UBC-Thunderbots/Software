// An obstacle is an area to avoid based on the size of the robot and its velocity
#pragma once

#include "ai/world/robot.h"
#include "geom/point.h"
#include "geom/polygon.h"
#include "geom/util.h"

class Obstacle
{
   public:
    /*
     * Gets the boundary polygon around the given robot obstacle that other robots
     * should not enter with a buffer scaled by radiusScaling and
     * velocityProjectionScaling
     * @param robot robot to create obstacle boundary polygon around
     * @param radiusScaling safety factor to vary the radius size of the buffer zone
     * (1=default), must greater than 0
     * @param velocityProjectionScaling scales the projection buffer in the direction of
     * the velocity (1=default)
     * @return a six-sided Polygon to represent the boundary around the obstacle
     */
    static Polygon getBoundaryPolygon(const Robot& robot, double robotRadiusScaling,
                                      double velocityProjectionScaling);
};
