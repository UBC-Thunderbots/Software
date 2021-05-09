#pragma once

/**
 * This struct represents a robot wheel/motor, from the perspective of firmware
 */
typedef struct VelocityWheel VelocityWheel_t;

/**
 * This struct holds wheel/motor constants
 */
typedef struct VelocityWheelConstants
{
    // The current per unit torque for the motor attached to this wheel [A/(N*m)]
    float motor_current_per_unit_torque;

    // The phase resistance for the motor attached to this wheel [Ohms]
    float motor_phase_resistance;

    // The back emf per motor rpm for the motor attached to this wheel [volt / rpm]
    float motor_back_emf_per_rpm;

    // The maximum voltage change that can be exerted on the motor attached to this
    // wheel before the wheel will slip [Volts]
    float motor_max_voltage_before_wheel_slip;

    // The radius of the wheel, in meters
    float wheel_radius;

    // The gear ratio between the motor shaft and wheel shaft
    // [# of wheel rotations / 1 motor rotation]
    float wheel_rotations_per_motor_rotation;
} VelocityWheelConstants_t;

/**
 * Create a wheel object with functions for interacting with it
 *
 * @param get_motor_speed_rpm A function that we can call to get the speed of this wheel,
 *                            in RPM
 * @param wheel_constants Constants for this wheel
 *
 * @return A pointer to the created wheel, ownership is given to the caller
 */
VelocityWheel_t* app_velocity_wheel_create(void (*set_target_rpm)(float),
                                           float (*get_motor_speed_rpm)(void),
                                           void (*brake)(void), void (*coast)(void),
                                           VelocityWheelConstants_t wheel_constants);

/**
 * Destroy the given wheel, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param wheel The wheel to destroy
 */
void app_velocity_wheel_destroy(VelocityWheel_t* wheel);

/**
 * Get the speed of the given wheel in RPM
 *
 * @param wheel The wheel to get the speed for
 *
 * @return The speed of the given wheel in RPM
 */
float app_velocity_wheel_getWheelSpeedRPM(VelocityWheel_t* wheel);

/**
 * Get the speed of the motor attached to the given wheel
 *
 * @param wheel The wheel to get the motor RPM for
 *
 * @return The speed of the motor attached to the given wheel, in RPM
 */
float app_velocity_wheel_getMotorSpeedRPM(const VelocityWheel_t* wheel);

/**
 * Allow this wheel to spin freely
 *
 * @param wheel The wheel to allow to spin freely
 */
void app_velocity_wheel_coast(const VelocityWheel_t* wheel);

/**
 * Brake this wheel
 *
 * @param wheel The wheel to brake
 */
void app_velocity_wheel_brake(const VelocityWheel_t* wheel);

/**
 * Set wheel to the target velocity. Positive velocity spins the wheel
 * counter-clockwise, as viewed from inside the robot looking out
 * (ie. induces positive angular velocity and rotation to the robot).
 *
 * @param wheel The wheel to set target rpm with
 * @param velocity_m_per_s The velocity in metres per second
 */
void app_velocity_wheel_setTargetVelocity(VelocityWheel_t* wheel, float velocity_m_per_s);

/**
 * Set wheel to the target rpm. Positive rpm spins the wheel
 * counter-clockwise, as viewed from inside the robot looking out
 * (ie. induces positive angular velocity and rotation to the robot).
 *
 * @param wheel The wheel to set target rpm with
 * @param rpm The rpm to set
 */
void app_velocity_wheel_setTargetRPM(VelocityWheel_t* wheel, float rpm);

/**
 * Get the constants for the given wheel
 *
 * @param wheel The wheel to get the constants for
 *
 * @return The constants for the given wheel
 */
VelocityWheelConstants_t app_velocity_wheel_getWheelConstants(
    const VelocityWheel_t* wheel);
