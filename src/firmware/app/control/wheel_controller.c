#include "firmware/app/control/wheel_controller.h"

#include "firmware/shared/circular_buffer.h"

struct WheelController
{
    CircularBuffer_t* previous_command_buffer;
    CircularBuffer_t* previous_output_sample_buffer;
    float* command_coefficients;
    float* output_sample_coefficients;
    size_t num_command_coefficients;
    size_t num_output_sample_coefficients;
};

WheelController_t* app_wheel_controller_create(
    float* command_coefficients, unsigned int num_command_coefficients,
    float* output_sample_coefficients, unsigned int num_output_sample_coefficients)
{
    // Create the circular buffers that will contain the sample and command history
    CircularBuffer_t* command_buffer = circular_buffer_create(num_command_coefficients);
    CircularBuffer_t* samples_buffer =
        circular_buffer_create(num_output_sample_coefficients);

    // Allocate memory to contain the coefficients of the difference equation
    float* internal_command_coefficients =
        (float*)malloc(sizeof(float) * num_command_coefficients);
    float* internal_samples_coefficients =
        (float*)malloc(sizeof(float) * num_output_sample_coefficients);

    // Allocate memory on the heap
    WheelController_t* wheel_controller =
        (WheelController_t*)malloc(sizeof(WheelController_t));

    // Copy the input coefficients to the internal arrays and
    // fill the circular buffers with zeros as the initial state
    for (size_t i = 0; i < num_command_coefficients; i++)
    {
        internal_command_coefficients[i] = command_coefficients[i];
        circular_buffer_push(command_buffer, 0.0f);
    }
    for (size_t i = 0; i < num_output_sample_coefficients; i++)
    {
        internal_samples_coefficients[i] = output_sample_coefficients[i];
        circular_buffer_push(samples_buffer, 0.0f);
    }

    // Point all of the wheel controller members to allocated memory
    wheel_controller->command_coefficients           = internal_command_coefficients;
    wheel_controller->output_sample_coefficients     = internal_samples_coefficients;
    wheel_controller->num_command_coefficients       = num_command_coefficients;
    wheel_controller->num_output_sample_coefficients = num_output_sample_coefficients;
    wheel_controller->previous_command_buffer        = command_buffer;
    wheel_controller->previous_output_sample_buffer  = samples_buffer;

    return wheel_controller;
}

void app_wheel_controller_pushNewCommand(WheelController_t* wheel_controller,
                                         float command)
{
    circular_buffer_push(wheel_controller->previous_command_buffer, command);
}

void app_wheel_controller_pushNewSampleOutput(WheelController_t* wheel_controller,
                                              float output_sample)
{
    circular_buffer_push(wheel_controller->previous_output_sample_buffer, output_sample);
}

float app_wheel_controller_getWheelVoltageToApply(WheelController_t* wheel_controller)
{
    float output_voltage = 0;

    // Get the contribution from previous commands
    for (size_t i = 0; i < wheel_controller->num_command_coefficients; i++)
    {
        output_voltage +=
            wheel_controller->command_coefficients[i] *
            circular_buffer_getAtIndex(wheel_controller->previous_command_buffer, i);
    }

    // Get the contribution from previous sampled outputs
    for (size_t i = 0; i < wheel_controller->num_output_sample_coefficients; i++)
    {
        output_voltage -= wheel_controller->output_sample_coefficients[i] *
                          circular_buffer_getAtIndex(
                              wheel_controller->previous_output_sample_buffer, i);
    }
    return output_voltage;
}

void app_wheel_controller_destroy(WheelController_t* wheel_controller)
{
    // Free all of the allocated memory
    free(wheel_controller->output_sample_coefficients);
    free(wheel_controller->command_coefficients);
    circular_buffer_destroy(wheel_controller->previous_command_buffer);
    circular_buffer_destroy(wheel_controller->previous_output_sample_buffer);

    free(wheel_controller);
}
