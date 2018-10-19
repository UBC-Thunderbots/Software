/**
 * This file contains the content for the grSim motion controller
 */

#pragma once

#include "util/constants.h"

namespace MotionController
{
    /**
     * Calculated robot velocities using Bang-bang motion controller for grSim
     *
     * @param robot The robot who's motion is to be controlled
     * @param dest The destination of the robot
     * @param desired_final_speed The target final speed of the robot
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * the robot (based on constant acceleration)
     * @return The linear velocity of the robot as a Vector(X,Y) and the angular velocity of the robot as a AngularVelocity packaged in a vector
     */
    std::pair<Vector, AngularVelocity> grSimBangBang(const Robot robot, const Point dest,
                                           const double desired_final_speed,
                                           const Angle desired_final_orientation,
                                           double delta_time);
}  // namespace MotionController
