#pragma once

#include "firmware/main/app/world/firmware_robot.h"

// TODO: need to prefix everything with "_app"

/**
 * Apply the given acceleration (in robot coordinates) to the given robot
 *
 * @param robot The robot to apply acceleration to
 * @param linear_accel_x The linear acceleration, in robot coordinates, in the x
 *                       (forward/backwards) direction (m/s^2)
 * @param linear_accel_y The linear acceleration, in robot coordinates, in the y
 *                       (side-to-side) direction (m/s^2)
 * @param angular_accel The angular acceleration to apply to the robot (rad/s^2)
 */
void app_control_applyAccel(const FirmwareRobot_t* robot, float linear_accel_x,
                            float linear_accel_y, float angular_accel);

/**
 * Drive the given robot at the given velocity in the global coordinate frame
 *
 * @param robot The robot to move at the given velocity
 * @param linear_velocity_x The velocity to move at in the x-direction in the global
 *                          coordinate frame (m/s)
 * @param linear_velocity_y The velocity to move at in the y-direction in the global
 *                          coordinate frame (m/s)
 * @param angular_velocity The angular velocity to move at (rad/s)
 */
void app_control_trackVelocity(FirmwareRobot_t* robot, float linear_velocity_x,
                               float linear_velocity_y, float angular_velocity);
