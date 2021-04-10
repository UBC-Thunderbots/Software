#pragma once

#include "firmware/app/world/force_wheels.h"
#include "firmware/app/world/firmware_robot_constants.h"

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
void app_control_applyAccel(const RobotConstants_t robot_constants, ControllerState_t* controller_state, 
                            float battery_voltage, ForceWheel_t* front_left_wheel,
                            ForceWheel_t* front_right_wheel, ForceWheel_t* back_left_wheel,
                            ForceWheel_t* back_right_wheel, float linear_accel_x,
                            float linear_accel_y, float angular_accel);
