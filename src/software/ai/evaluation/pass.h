#pragma once

#include "software/geom/angle.h"
#include "software/geom/point.h"
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
Duration getTimeToOrientationForRobot(
    const Angle& current_orientation, const Angle& desired_orientation,
    const double& max_velocity, const double& max_acceleration,
    const AngularVelocity& initial_angular_velocity = AngularVelocity::zero());

/**
 * Calculate minimum time it would take for the given robot to reach the given point
 *
 * The calculations are done in 1D to simplify calculations and be as performant as
 * possible
 *
 * @param start The starting point of robot
 * @param dest The destination that the robot is going to
 * @param max_velocity The maximum linear velocity the robot can travel at (m/s)
 * @param max_acceleration The maximum acceleration of the robot (m/s^2)
 * @param tolerance_meters The radius around the target at which we will be considered
 *                         "at" the target.
 * @param initial_velocity The initial velocity of the robot
 * @param final_velocity The desired final velocity which the robot should be moving at
 *
 * @return The minimum theoretical time it would take the robot to reach the dest
 * point
 */
Duration getTimeToPositionForRobot(const Point& start, const Point& dest,
                                   const double max_velocity,
                                   const double max_acceleration,
                                   const double tolerance_meters  = 0,
                                   const Vector& initial_velocity = Vector(),
                                   const Vector& final_velocity   = Vector());

/**
 * Calculate minimum time it would take for an robot to move a set distance
 *
 * The calculations are done in 1D to simplify calculations and be as performant as
 * possible
 *
 * @param distance The distance which the robot is moving
 * @param max_velocity The maximum linear velocity the robot can travel at (m/s)
 * @param max_acceleration The maximum acceleration of the robot (m/s^2)
 * @param initial_velocity The initial velocity of the robot (m/s)
 * @param final_velocity The desired final velocity which the robot should be moving at
 * (m/s)
 *
 * @return The minimum theoretical time it would take the robot to reach the dest
 * point
 */
Duration getTimeToTravelDistance(const double distance, const double max_velocity,
                                 const double max_acceleration,
                                 const double initial_velocity = 0,
                                 const double final_velocity   = 0);
