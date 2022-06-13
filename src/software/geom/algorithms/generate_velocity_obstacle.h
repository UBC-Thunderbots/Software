#pragma once

#include <vector>

#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/circle.h"
#include "software/geom/polygon.h"

/**
 * Set of functions that generate velocity obstacles for an obstacle
 * NOTE: These functions go off of the assumption that the robot (circle origin)
 *       is not contained within the obstacle. If it is, the output of the
 *       function will be undefined.
 *
 * @param obstacle Static obstacle
 * @param robot The robot which the velocity obstacle will be for, represented
 * with a circle
 * @param obstacle_velocity The velocity which the obstacle is moving at
 * @return Velocity obstacle generated for the robot, relative to the obstacle
 */
VelocityObstacle generateVelocityObstacle(const Circle& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity);
VelocityObstacle generateVelocityObstacle(const Polygon& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity);
