#pragma once

#include <vector>

#include "software/ai/navigator/path_planner/hrvo/velocity_obstacle.h"
#include "software/geom/circle.h"
#include "software/geom/polygon.h"

/**
 * Set of functions that generate velocity obstacles for an obstacle
 *
 * @param obstacle The obstacle which the robot is trying to avoid
 * @param robot The robot which the velocity obstacle will be for, represented
 * with a circle
 * @param obstacle_velocity The velocity which the obstacle is moving at
 * @return Velocity obstacle generated for the robot, relative to the obstacle
 */
VelocityObstacle generateVelocityObstacle(const Circle& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity);
/*
 * NOTE: The polygon implementation may return incorrect velocity obstacle if:
 *       - the robot is intersecting or is contained by the obstacle
 *       - the polygon is concave
 */
VelocityObstacle generateVelocityObstacle(const Polygon& obstacle, const Circle& robot,
                                          const Vector& obstacle_velocity);
