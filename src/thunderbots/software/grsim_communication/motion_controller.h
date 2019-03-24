/**
 * This file contains the content for the grSim motion controller
 */

#pragma once

#include <variant>

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"

class MotionController
{
   public:
    MotionController(const double max_speed_meters_per_second,
                     const double max_angular_speed_radians_per_second,
                     const double max_acceleration_meters_per_second_squared,
                     const double max_angular_acceleration_meters_per_second_squared)
        : max_speed_meters_per_second(max_speed_meters_per_second),
          max_angular_speed_radians_per_second(max_angular_speed_radians_per_second),
          max_acceleration_meters_per_second_squared(
              max_acceleration_meters_per_second_squared),
          max_angular_acceleration_meters_per_second_squared(
              max_angular_acceleration_meters_per_second_squared)

    {
    }

    struct Velocity
    {
        Vector linear_velocity;
        AngularVelocity angular_velocity;
    };

    /**
     * Structure for a MotionControllerCommand which determines a velocity based on final
     * position.
     */
    struct PositionCommand
    {
        PositionCommand()
            : global_destination(Vector(0, 0)),
              final_orientation(Angle::zero()),
              final_speed_at_destination(0.0),
              kick_speed_meters_per_second(0.0),
              chip_instead_of_kick(false),
              dribbler_on(false)
        {
        }

        PositionCommand(Vector global_destination, Angle final_orientation,
                        double final_speed_at_destination, double kick_or_chip_power,
                        bool chip_instead_of_kick, bool dribbler_on)
            : global_destination(global_destination),
              final_orientation(final_orientation),
              final_speed_at_destination(final_speed_at_destination),
              kick_speed_meters_per_second(kick_or_chip_power),
              chip_instead_of_kick(chip_instead_of_kick),
              dribbler_on(dribbler_on)
        {
        }

        PositionCommand(Angle final_orientation, double kick_or_chip_power,
                        bool chip_instead_of_kick, bool dribbler_on,
                        Vector requested_linear_velocity,
                        AngularVelocity requested_angular_velocity)
            : global_destination(Vector(0, 0)),
              final_orientation(final_orientation),
              final_speed_at_destination(0.0),
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

    /**
     * Structure for a MotionControllerCommand which determines velocity directly through
     * the provided velocities
     */
    struct VelocityCommand
    {
        VelocityCommand(double kick_or_chip_power, bool chip_instead_of_kick,
                        bool dribbler_on, Vector linear_velocity,
                        AngularVelocity angular_velocity)
            : kick_speed_meters_per_second(kick_or_chip_power),
              chip_instead_of_kick(chip_instead_of_kick),
              dribbler_on(dribbler_on),
              linear_velocity(linear_velocity),
              angular_velocity(angular_velocity)
        {
        }


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
        // Whether the caller is directly requesting a velocity instead of position
        Vector linear_velocity;
        // Outputted angular velocity when the caller wants to directly request velocity
        // instead of a position.
        AngularVelocity angular_velocity;
    };

    // tolerance distance measurement in meters
    const double VELOCITY_STOP_TOLERANCE       = 0.02;
    const double POSITION_STOP_TOLERANCE       = 0.01;
    const Angle DESTINATION_VELOCITY_TOLERENCE = Angle::ofDegrees(1);

    // Constants used to determine output velocity of the controller
    const double max_speed_meters_per_second;
    const double max_angular_speed_radians_per_second;
    const double max_acceleration_meters_per_second_squared;
    const double max_angular_acceleration_meters_per_second_squared;

    /**
     * Calculate new robot velocities based on current robot state and destination
     * criteria or with the provided velocities using Bang-bang motion controller
     *
     * @param robot The robot whose motion is to be controlled
     * @param delta_time The change in time since the motion controller was run last
     * @param motion_command A variant which contains either a Velocity or Position
     * command
     * @return The linear velocity of the robot as a Vector(X,Y) and the angular velocity
     * of the robot as a AngularVelocity packaged in a vector
     */
    Velocity bangBangVelocityController(
        Robot robot, const double delta_time,
        std::variant<MotionController::PositionCommand, MotionController::VelocityCommand>
            motion_command);

    /**
     * Calculate robot angular velocities based on current robot polar state and
     * destination polar criteria
     *
     * @param robot The robot whose motion is to be controlled
     * @param desired_final_orientation The target final orientation of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * @return Angular velocity of the robot as a AngularVelocity packaged in a vector
     */
    AngularVelocity determineAngularVelocityFromPosition(
        const Robot robot, const Angle desired_final_orientation, const double delta_time,
        const double max_angular_speed_radians_per_second,
        const double max_angular_acceleration_radians_per_second_squared);

    /**
     * Calculate robot linear velocities based on current robot cartesian state and
     * destination cartesian state
     *
     * @param robot The robot whose motion is to be controlled
     * @param desired_final_speed The target final speed of the robot
     * @param delta_time The time that will be used to calculate the change in speed of
     * @return Linear velocity of the robot as a AngularVelocity packaged in a vector
     */
    Vector determineLinearVelocityFromPosition(
        const Robot robot, const Point dest, const double desired_final_speed,
        const double delta_time, const double max_speed_meters_per_second,
        const double max_acceleration_meters_per_second_squared);

    /**
     *
     * @param robot The robot whose motion is to be controlled
     * @param linear_velocity The target linear velocity
     * @param max_acceleration_meters_per_second_squared
     * @param delta_time The time that will be used to calculate the change in speed
     * @return Linear velocity of the robot as a vector.
     */
    Vector determineLinearVelocityFromVelocity(
        const Robot robot, const Vector linear_velocity,
        const double max_acceleration_meters_per_second_squared, const double delta_time,
        const double max_speed_meters_per_second);

    /**
     *
     * @param robot The robot whose motion is to be controlled
     * @param angular_velocity The target angular velocity
     * @param max_angular_acceleration_meters_per_second_squared
     * @param delta_time
     * @return
     */
    AngularVelocity determineAngularVelocityFromVelocity(
        const Robot robot, AngularVelocity angular_velocity,
        const double max_angular_acceleration_meters_per_second_squared,
        const double delta_time);

};  // class MotionController
