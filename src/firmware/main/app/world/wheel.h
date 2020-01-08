#pragma once

// TODO: unit tests for new functionality

/**
 * This struct represents a robot wheel/motor, from the perspective of firmware
 */
typedef struct Wheel Wheel_t;

// TOOD: should it be "wheelconstants" or "physical wheel constants"? same for
// function(s)?
/**
 * This struct holds wheel/motor constants
 */
typedef struct WheelConstants
{
    // The current per unit torque for the motor attached to this wheel [A/(n*m)]
    float motor_current_per_unit_torque;

    // The phase resistance for the motor attached to this wheel [Ohms]
    float motor_phase_resistance;

    // The back emf per motor rpm for the motor attached to this wheel [rpm / volt]
    float motor_back_emf_per_rpm;

    // The maximum voltage change that can be exerted on the motor attached to this
    // wheel before the wheel will slip [Volts]
    float motor_max_delta_voltage_before_wheel_slip;

    float wheel_radius;

    // The gear ratio between the motor shaft and wheel shaft
    // [# of motor rotations / # of wheel rotations]
    float motor_rotations_per_wheel_rotation;
} WheelConstants_t;

/**
 * Create a wheel object with functions for interacting with it
 *
 * @param apply_wheel_force A function that we can call to apply a force to this wheel,
 *                          in newtons
 * @param get_wheel_speed_rpm A function that we can call to get the speed of this wheel,
 *                            in RPM
 * @param wheel_constants Constants for this wheel
 *
 * @return A pointer to the created wheel, ownership is given to the caller
 */
Wheel_t* app_wheel_create(void (*apply_wheel_force)(float force_in_newtons),
                          float (*get_wheel_speed_rpm)(),
                          WheelConstants_t wheel_constants);

/**
 * Destroy the given wheel, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param wheel The wheel to destroy
 */
void app_wheel_destroy(Wheel_t* wheel);

/**
 * Apply the given force to the given wheel
 * @param wheel The wheel to apply force to
 * @param force_in_newtons The force to apply to the wheel, in newtons
 */
void app_wheel_applyForce(Wheel_t* wheel, float force_in_newtons);

/**
 * Get the speed of the given wheel in RPM
 * @param wheel The wheel to get the speed for
 * @return The speed of the given wheel in RPM
 */
float app_wheel_getWheelSpeedRPM(Wheel_t* wheel);

// TODO: implement and test me
/**
 * Get the speed of the motor attached to the given wheel
 * @param wheel The wheel to get the motor RPM for
 * @return The speed of the motor attached to the given wheel, in RPM
 */
float app_wheel_getMotorSpeedRPM(Wheel_t* wheel);

/**
 * Get the constants for the given wheel
 * @param wheel The wheel to get the constants for
 * @return The constants for the given wheel
 */
const WheelConstants_t app_wheel_getWheelConstants(Wheel_t* wheel);
