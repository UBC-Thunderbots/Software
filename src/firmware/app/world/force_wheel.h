#pragma once

#include "shared/robot_constants.h"

/**
 * This struct represents a robot wheel/motor, from the perspective of firmware
 */
typedef struct ForceWheel ForceWheel_t;

/**
 * Create a wheel object with functions for interacting with it
 *
 * @param apply_wheel_force A function that we can call to apply a force to this wheel,
 *                          in newtons
 * @param get_motor_speed_rpm A function that we can call to get the speed of this wheel,
 *                            in RPM
 * @param wheel_constants Constants for this wheel
 *
 * @return A pointer to the created wheel, ownership is given to the caller
 */
ForceWheel_t* app_force_wheel_create(void (*apply_wheel_force)(float),
                                     float (*get_motor_speed_rpm)(void),
                                     void (*brake)(void), void (*coast)(void),
                                     WheelConstants_t wheel_constants);

/**
 * Destroy the given wheel, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param wheel The wheel to destroy
 */
void app_force_wheel_destroy(ForceWheel_t* wheel);

/**
 * Apply the given force to the given wheel
 *
 * @param wheel The wheel to apply force to
 *
 * @param force_in_newtons The force to apply to the wheel, in newtons
 */
void app_force_wheel_applyForce(ForceWheel_t* wheel, float force_in_newtons);

/**
 * Get the speed of the given wheel in RPM
 *
 * @param wheel The wheel to get the speed for
 *
 * @return The speed of the given wheel in RPM
 */
float app_force_wheel_getWheelSpeedRPM(ForceWheel_t* wheel);

/**
 * Get the speed of the motor attached to the given wheel
 *
 * @param wheel The wheel to get the motor RPM for
 *
 * @return The speed of the motor attached to the given wheel, in RPM
 */
float app_force_wheel_getMotorSpeedRPM(const ForceWheel_t* wheel);

/**
 * Allow this wheel to spin freely
 *
 * @param wheel The wheel to allow to spin freely
 */
void app_force_wheel_coast(const ForceWheel_t* wheel);

/**
 * Brake this wheel
 *
 * @param wheel The wheel to brake
 */
void app_force_wheel_brake(const ForceWheel_t* wheel);

/**
 * Get the constants for the given wheel
 *
 * @param wheel The wheel to get the constants for
 *
 * @return The constants for the given wheel
 */
WheelConstants_t app_force_wheel_getWheelConstants(const ForceWheel_t* wheel);
