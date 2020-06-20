#pragma once

#include "firmware/shared/circular_buffer.h"

#define CONTROLLER_TICK_TIME 0.010  // seconds

// wheel_controller_difference_equation.h
typedef struct WheelController{
    CircularBuffer_t* previous_command_buffer;
    CircularBuffer_t* previous_output_samples_buffer;
    float* command_coefficients;
    float* sampled_output_coefficients;
    unsigned int num_command_coefficients;
    unsigned int num_samples_coefficients;
} WheelController_t;


WheelController_t* app_wheel_controller_create(float* command_coefficients, unsigned int num_command_coefficients, float* sampled_output_coefficients, unsigned int num_sample_coefficients){

    // Create the circular buffers that will contain the sample and command history
    CircularBuffer_t* command_buffer = circular_buffer_create(num_command_coefficients);
    CircularBuffer_t* samples_buffer = circular_buffer_create(num_sample_coefficients);

    // Allocate memory to contain the coefficients of the difference equation
    float*  internal_command_coefficients = (float*)malloc(sizeof(float)*num_command_coefficients);
    float* internal_samples_coefficients = (float*)malloc(sizeof(float)*num_sample_coefficients);

    // Allocate memory on the heap
    WheelController_t* wheel_controller = (WheelController_t*)malloc(sizeof(WheelController_t));

    // Copy the input coefficients to the internal arrays and
    // fill the circular buffers with zeros as the initial state
    for(unsigned int i = 0; i < num_command_coefficients; i++ ){
        internal_command_coefficients[i] = command_coefficients[i];
        circular_buffer_push(command_buffer, 0.0f);
    }
    for(unsigned int i = 0; i < num_sample_coefficients; i++ ){
        internal_samples_coefficients[i] = sampled_output_coefficients[i];
        circular_buffer_push(samples_buffer, 0.0f);
    }

    // Point all of the wheel controllers to allocated memory
    wheel_controller->command_coefficients = internal_command_coefficients;
    wheel_controller->sampled_output_coefficients = internal_samples_coefficients;
    wheel_controller->num_command_coefficients = num_command_coefficients;
    wheel_controller->num_samples_coefficients = num_sample_coefficients;
    wheel_controller->previous_command_buffer = command_buffer;
    wheel_controller->previous_output_samples_buffer = samples_buffer;
}

app_wheel_controller_pushNewCommand(WheelController_t* wheel_controller, float command){

    circular_buffer_push(wheel_controller->previous_command_buffer, command);
}

app_wheel_controller_pushNewSampleOutput(WheelController_t* wheel_controller, float output_sample){
    circular_buffer_push(wheel_controller->previous_output_samples_buffer, output_sample);
}

app_wheel_controller_getWheelVoltage(WheelController_t* wheel_controller){

    float output_voltage = 1;

    // Get the contribution from previous commands
    for(int i = 0; i < (int)wheel_controller->num_command_coefficients; i++){
        output_voltage += wheel_controller->command_coefficients[i] * circular_buffer_getAtIndex(wheel_controller->previous_command_buffer, (size_t)i+1);
    }

    // Get the contribution from previous sampled outputs
    for(int i = 0; i < (int)wheel_controller->num_samples_coefficients; i++){
        output_voltage -= wheel_controller->sampled_output_coefficients[i] * circular_buffer_getAtIndex(wheel_controller->previous_output_samples_buffer, (size_t)i+1);
    }
}

void app_wheel_controller_destroy(WheelController_t* wheel_controller){

    // Free all of the allocated memory
    free(wheel_controller->previous_output_samples_buffer);
    free(wheel_controller->previous_command_buffer);
    free(wheel_controller->sampled_output_coefficients);
    free(wheel_controller->command_coefficients);
}
