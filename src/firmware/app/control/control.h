#pragma once

#include "firmware/app/world/force_wheel.h"
#include "shared/robot_constants.h"

/**
 * Apply the given acceleration (in robot coordinates) to the given robot
 *
 * @param robot_constants The robot constants representing the robot
 * @param controller_state The controller state representing the robot
 * @param battery_voltage The robot's battery voltage
 * @param force_wheels The robot's force wheels (front_left, back_left, back_right,
 * front_right in order)
 * @param linear_accel_x The linear acceleration, in robot coordinates, in the x
 *                       (forward/backwards) direction (m/s^2)
 * @param linear_accel_y The linear acceleration, in robot coordinates, in the y
 *                       (side-to-side) direction (m/s^2)
 * @param angular_accel The angular acceleration to apply to the robot (rad/s^2)
 */
void app_control_applyAccel(const RobotConstants_t robot_constants,
                            ControllerState_t* controller_state, float battery_voltage,
                            ForceWheel_t* force_wheels[4], float linear_accel_x,
                            float linear_accel_y, float angular_accel);
