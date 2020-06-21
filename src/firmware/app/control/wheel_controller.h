
#include "firmware/shared/circular_buffer.h"

// wheel_controller_difference_equation.h
typedef struct WheelController
{
    CircularBuffer_t* previous_command_buffer;
    CircularBuffer_t* previous_output_samples_buffer;
    float* command_coefficients;
    float* sampled_output_coefficients;
    unsigned int num_command_coefficients;
    unsigned int num_samples_coefficients;
} WheelController_t;

/**
 *
 * @param command_coefficients
 * @param num_command_coefficients
 * @param sampled_output_coefficients
 * @param num_sample_coefficients
 * @return
 */
WheelController_t* app_wheel_controller_create(float* command_coefficients,
                                               unsigned int num_command_coefficients,
                                               float* sampled_output_coefficients,
                                               unsigned int num_sample_coefficients);
/**
 *
 * @param wheel_controller
 * @param command
 */
void app_wheel_controller_pushNewCommand(WheelController_t* wheel_controller, float command);

/**
 *
 * @param wheel_controller
 * @param output_sample
 */
void app_wheel_controller_pushNewSampleOutput(WheelController_t* wheel_controller,
float output_sample);

/**
 *
 * @param wheel_controller
 * @return
 */
float app_wheel_controller_getWheelVoltage(WheelController_t* wheel_controller);

/**
 *
 * @param wheel_controller
 */
void app_wheel_controller_destroy(WheelController_t* wheel_controller);