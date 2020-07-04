#pragma once

#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/world/robot.h"

/**
 * General evaluation functions for related to Passes and Passing, that are also used
 * across multiple parts of AI
 */

/**
 * Calculate how long it would take the given robot to turn to the given orientation
 *
 * @param current_orientation The current orientation of the robot
 * @param desired_orientation The orientation which we want the robot to be at
 * @param max_velocity The maximum angular velocity that robot can turn at (rad/s)
 * @param max_acceleration The maximum angular rate at which the robot can
 *                             accelerate (rad/s^2)
 *
 * @return The time required for the given robot to rotate to the given orientation
 */
Duration getTimeToOrientationForRobot(const Angle& current_orientation,
                                      const Angle& desired_orientation,
                                      const double& max_velocity,
                                      const double& max_acceleration);

/**
 * Calculate minimum time it would take for the given robot to reach the given point
 *
 * This is only a rough calculation in order to be as performant as possible
 *
 * @param start The starting point of robot
 * @param dest The destination that the robot is going to
 * @param max_velocity The maximum linear velocity the robot can travel at (m/s)
 * @param max_acceleration The maximum acceleration of the robot (m/s^2)
 * @param tolerance_meters The radius around the target at which we will be considered
 *                         "at" the target.
 *
 * @return The minimum theoretical time it would take the robot to reach the dest
 * point
 */
Duration getTimeToPositionForRobot(const Point& start, const Point& dest,
                                   const double max_velocity,
                                   const double max_acceleration,
                                   const double tolerance_meters = 0);
