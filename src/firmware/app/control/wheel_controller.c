#pragma once

#include "firmware/shared/circular_buffer.h"

#define CONTROLLER_TICK_TIME 0.010  // seconds

// wheel_controller_difference_equation.h
typedef struct WheelController{
    CircularBuffer_t* previous_command_buffer;
    CircularBuffer_t* previous_output_samples_buffer;
    float* input_coefficients;
    float* output_coefficients;
    unsigned int num_command_coefficients;
    unsigned int num_samples_coefficients;
} WheelController_t;


WheelController_t* app_wheel_controller_create(float* command_coefficients, unsigned int num_command_coefficients, float* samples_coefficients, unsigned int num_sample_coefficients){

    // Create the circular buffers that will contain the sample and command history
    CircularBuffer_t* command_buffer = circular_buffer_create(samples_coefficients);
    CircularBuffer_t* samples_buffer = circular_buffer_create(samples_coefficients);

    // Allocate memory to contain the coefficients of the difference equation
    float*  internal_command_coefficients = (float*)malloc(sizeof(float)*num_command_coefficients);
    float* internal_samples_coefficients = (float*)malloc(sizeof(float)*num_sample_coefficients);

    // Copy the input coefficients to the internal arrays
    for(unsigned int i = 0; i < num_command_coefficients; i++ ){
        internal_command_coefficients[i] = command_coefficients[i];
    }
    for(unsigned int i = 0; i < num_sample_coefficients; i++ ){
        internal_samples_coefficients[i] = samples_coefficients[i];
    }

    // Allocate memory on the heap
    WheelController_t* wheel_controller = (WheelController_t*)malloc(sizeof(WheelController_t));

    // Point all of the wheel controllers to allocated memory
    wheel_controller->input_coefficients = internal_command_coefficients;
    wheel_controller->output_coefficients = internal_samples_coefficients;
    wheel_controller->num_command_coefficients = num_command_coefficients;
    wheel_controller->num_samples_coefficients = num_sample_coefficients;
    wheel_controller->previous_command_buffer = command_buffer;
    wheel_controller->previous_output_samples_buffer = samples_buffer;
}

void app_wheel_controller_destroy(WheelController_t* wheel_controller){

    // Free all of the allocated memory
    free(wheel_controller->previous_output_samples_buffer);
    free(wheel_controller->previous_command_buffer);
    free(wheel_controller->output_coefficients);
    free(wheel_controller->input_coefficients);
}
void app_wheel_controller_destroy(WheelController_t* wheel_controller);
typedef struct WheelControllerDiffEqnCoeffs
{
    float* wheel_speed_coefficients;
    size_t num_wheel_speed_coefficients;
    float* previous_wheel_speed_command_coefficients;
    size_t num_previous_wheel_speed_command_coefficients;
} WheelControllerDiffEqn_t;

typedef enum
{
    WHEEL_SPEED_CONTROLLER_DIFF_EQN_OK,
    WHEEL_SPEED_CONTROLLER_DIFF_EQN_NOT_ENOUGH_INPUTS,
    WHEEL_SPEED_CONTROLLER_DIFF_EQN_NOT_ENOUGH_COEFFS,
} WheelSpeedControllerDiffEqnStatus;
/**
 *
 * @param actual_wheel_speed_history [in] A buffer of previously *observed* wheel speeds,
 *                                        sampled at `WHEEL_SPEED_CONTROLLER_TICK_TIME`
 *                                        [rad/s]
 * @param speed_command_history [in] A buffer of previously commanded wheel speeds,
 *                                   sent at `WHEEL_SPEED_CONTROLLER_TICK_TIME`
 *                                   intervals. The value at the front of this buffer is
 *                                   the currently desired wheel speed. [rad/s]
 * @param coefficients [in] The coefficients that define the difference equation
 * @param voltage_to_apply [out] This will be set to the voltage to apply to
 *                               the wheels to obtain the desired speed, *if*
 *                               the controller is run successfully. Otherwise
 *                               is set to 0.
 *
 * @return WHEEL_SPEED_CONTROLLER_DIFF_EQN_OK if the difference equation was
 *                                            able to run successfully, otherwise
 *                                            an error code indicating the
 *                                            issue that occured.
 */
WheelSpeedControllerDiffEqnStatus get_voltage(
    CircularBuffer_t* actual_wheel_speed_history,
    CircularBuffer_t* speed_command_history float* voltage_to_apply);
// controller_difference_equation.c
WheelSpeedControllerDiffEqnStatus get_voltage(
    CircularBuffer_t* actual_wheel_speed_history,
    CircularBuffer_t* speed_command_history float* voltage_to_apply)
{
    // Here we need to:
    // - make sure that there are enough inputs
    // - if there are, use the generated difference equation to create the
    //   output voltage
}