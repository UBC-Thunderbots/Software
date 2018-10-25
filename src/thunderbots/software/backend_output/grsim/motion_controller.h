/**
 * This file contains the content for the grSim motion controller
 */

#pragma once

#include "util/constants.h"

namespace MotionController
{
    struct Velocity {
        Vector linear_velocity;
        AngularVelocity angular_velocity;
    };

    /**
     * Calculate robot velocities using Bang-bang motion controller
     *
     * @param robot The robot whose motion is to be controlled
     * @param dest The destination of the robot
     * @param desired_final_speed The target final speed of the robot
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * the robot (based on constant acceleration)
     * @return The linear velocity of the robot as a Vector(X,Y) and the angular velocity of the robot as a AngularVelocity packaged in a vector
     */
    Velocity bangBangVelocityController(const Robot robot, const Point dest,
                                                                  const double desired_final_speed,
                                                                  const Angle desired_final_orientation,
                                                                  const double delta_time);

    /**
     * Calculate robot angular velocities
     *
     * @param robot The robot whose motion is to be controlled
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * @return Angular velocity of the robot as a AngularVelocity packaged in a vector
     */
    AngularVelocity determineAngularVelocity(const Robot robot, const Angle desired_final_orientation, const double delta_time);

    /**
    * Calculate robot linear velocities
    *
    * @param robot The robot whose motion is to be controlled
    * @param desired_final_speed The target final speed of the robot
    * @param delta_time The time that will be used to calculate the change in speed of
    * @return Linear velocity of the robot as a AngularVelocity packaged in a vector
    */
    Vector determineLinearVelocity(const Robot robot, const Point dest, const double desired_final_speed, const double delta_time);

}  // namespace MotionController
