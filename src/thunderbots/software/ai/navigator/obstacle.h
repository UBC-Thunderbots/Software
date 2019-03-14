// An obstacle is an area to avoid based on the size of the robot and its velocity
#pragma once

#include "geom/polygon.h"
#include "geom/point.h"
#include "geom/util.h"
#include "util/parameter/dynamic_parameters.h"

class Obstacle
{
   public:
    /*
     * Gets the buffer zone around the given robot obstacle that other robots
     * should not enter
     * @param robot robot to create obstacle buffer zone around
     * @param avoid_factor safety factor to vary the size of the buffer zone (1=default)
     * @return a six-sided Polygon to represent the buffer zone around the obstacle
     */
    static Polygon getBufferZone(const Robot& robot, double avoid_factor);
};
