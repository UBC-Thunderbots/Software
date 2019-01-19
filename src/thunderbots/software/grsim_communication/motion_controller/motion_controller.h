/**
 * This file contains the content for the grSim motion controller
 */

#pragma once

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"

namespace MotionController
{
    struct Velocity
    {
        Vector linear_velocity;
        AngularVelocity angular_velocity;
    };

    struct MotionControllerCommand
    {
        MotionControllerCommand()
            : global_destination(Vector(0, 0)),
              final_orientation(Angle::zero()),
              final_speed_at_destination(0.0),
              kick_speed_meters_per_second(0.0),
              chip_instead_of_kick(false),
              dribbler_on(false)
        {
        }

        MotionControllerCommand(Vector global_destination, Angle final_orientation,
                                double final_speed_at_destination,
                                double kick_or_chip_power, bool chip_instead_of_kick,
                                bool dribbler_on)
            : global_destination(global_destination),
              final_orientation(final_orientation),
              final_speed_at_destination(final_speed_at_destination),
              kick_speed_meters_per_second(kick_or_chip_power),
              chip_instead_of_kick(chip_instead_of_kick),
              dribbler_on(dribbler_on)
        {
        }

        // The point (in global coordinates) the robot should move towards in a
        // straight line
        Vector global_destination;
        // The global orientation the robot should have when it arrives at the destination
        Angle final_orientation;
        // The speed the robot should have when it arrives at the destination
        double final_speed_at_destination;
        // How fast to kick the ball when the robot makes contact with it. A value of
        // 0 means the robot will not kick/chip the ball. Whether or not the robot kicks
        // or chips the ball is controlled by the 'chip_instead_of_kick' parameter below
        double kick_speed_meters_per_second;
        // Whether or not the robot should chip the ball instead of kick it when it makes
        // contact with the ball. If this value is false, the robot will kick the ball.
        // If it is true, the robot will kick the ball.
        bool chip_instead_of_kick;
        // Whether or not the robot's dribbler should be on. Dribbler speed cannot be
        // controlled in grSim
        bool dribbler_on;
    };

    // tolerance distance measurement in meters
    const double VELOCITY_STOP_TOLERANCE       = 0.02;
    const double POSITION_STOP_TOLERANCE       = 0.01;
    const Angle DESTINATION_VELOCITY_TOLERENCE = Angle::ofDegrees(1);

    /**
     * Calculate new robot velocities based on current robot state and destination
     * criteria using Bang-bang motion controller
     *
     * @param robot The robot whose motion is to be controlled
     * @param dest The destination of the robot
     * @param desired_final_speed The target final speed of the robot
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The change in time since the motion controller was run last
     * @return The linear velocity of the robot as a Vector(X,Y) and the angular velocity
     * of the robot as a AngularVelocity packaged in a vector
     */
    Velocity bangBangVelocityController(const Robot robot, const Point dest,
                                        const double desired_final_speed,
                                        const Angle desired_final_orientation,
                                        const double delta_time);

    /**
     * Calculate robot angular velocities based on current robot polar state and
     * destination polar criteria
     *
     * @param robot The robot whose motion is to be controlled
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * @return Angular velocity of the robot as a AngularVelocity packaged in a vector
     */
    AngularVelocity determineAngularVelocity(const Robot robot,
                                             const Angle desired_final_orientation,
                                             const double delta_time);

    /**
     * Calculate robot linear velocities based on current robot cartesian state and
     * destination cartesian state
     *
     * @param robot The robot whose motion is to be controlled
     * @param desired_final_speed The target final speed of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * @return Linear velocity of the robot as a AngularVelocity packaged in a vector
     */
    Vector determineLinearVelocity(const Robot robot, const Point dest,
                                   const double desired_final_speed,
                                   const double delta_time);

}  // namespace MotionController
