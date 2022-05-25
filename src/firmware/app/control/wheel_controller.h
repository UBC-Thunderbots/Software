#pragma once

#include "firmware/shared/circular_buffer.h"

typedef struct WheelController WheelController_t;

/*
 * NOTE: THIS FUNCTIONS IN THIS FILE OPERATE UNDER THE ASSUMPTION THAT THEY ARE BEING USED
 * IN A DISCRETE TIME CONTROLLED SYSTEM. THIS MEANS THAT FOR A CONSTANT INTERPOLATION
 * PERIOD OF 'T' ONLY ONE DATA SAMPLE IS ADDED TO THE SAMPLE BUFFER AND ONLY ONE COMMAND
 * IS ADDED TO THE COMMAND BUFFER.
 *
 * THIS MEANS THAT OVER THE PERIOD OF 'T' SECONDS THERE IS NO CHANGE TO THE APPLIED
 * VOLTAGE. THIS IS KNOWN AS 'ZERO-ORDER-HOLD' (ZOH).
 *
 * Example Usage:
 *
 *  while(trajectory_is_not_done) {
 *
 *      wheel_speed = sampleEncoder();
 *      new_wheel_speed = getNextWheelSpeedFromPlanner();
 *      pushNewCommand(new_wheel_speed);
 *      pushSampledOutput(wheel_speed);
 *
 *      new_voltage_to_apply = getWheelVoltageToApply();
 *      applyMotorVoltage(new_voltage_to_apply)
 *
 *      sleep(INTERPOLATION_PERIOD)
 *   }
 *
 * For more information:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/dynamic-systems-n-control/idsc-dam/Lectures/Digital-Control-Systems/Slides_DigReg_2013.pdf
 *
 */

/**
 * Function creates a WheelController_t with the specified parameters and returns a
 * pointer to it. The wheel_controller_destroy function MUST be called before the
 * WheelController_t* goes out of scope.
 * The WheelController_t allows previously sampled wheel states to be added to it's state,
 * and allows for new wheel commands to be requested.
 *
 * The coefficients parameters are in order of decreasing recentness - where
 * coefficient_array[0] is the most recent value.
 *
 * The convention is as follows:
 * Where 'z' is the discrete time operator and z^M is equivalent to M time-steps BACKWARDS
 * in time. coeff[0] -> z^0 coeff[1] -> z^-1 coeff[2] -> z^-2
 * ...
 * coeff[N] -> z^-N
 *
 * NOTE: All coefficients MUST be specified. That is if the difference equation is A*z^-3
 * + B*z^-1 the z^-2 coefficient MUST be included. Ex, A*z^-3 + 0*z^-2 + B*z^-1.
 *
 * The difference equation created is:
 *      Output_voltage = Command_coefficients*Command_values -
 * Output_sample_coefficients*output_values
 *
 * @param command_coefficients [in] The coefficients for the controller command inputs.
 * These are the coefficients relating to how previous input effects the controller
 * output.
 *
 * @param num_command_coefficients The number of elements in the command_coefficients
 * array
 *
 * @param output_sample_coefficients [in] The coefficients for the controller sampled
 * output. These are the coefficients relating to how the sampled output(wheel state)
 * effects the controller output.
 *
 * @param num_output_sample_coefficients The number of elements in the
 * sampled_output coefficients array.
 *
 * @return Pointer to the newly created WheelController_t
 */
WheelController_t* app_wheel_controller_create(
    float* command_coefficients, unsigned int num_command_coefficients,
    float* output_sample_coefficients, unsigned int num_output_sample_coefficients);

/**
 * Function pushes a new command value to the specified WheelController. The command
 * SHOULD have the same units as the desired output. Ex. Command = 1m/s means that the
 * user desires a wheel speed of 1m/s.
 *
 * @param wheel_controller [in] The WheelController that is the target of the new command
 * value.
 *
 * @param command The value of the commanded state requested. This is the desired
 * state of the wheel. The units are the same as the desired output. Ex, m/s.
 */
void app_wheel_controller_pushNewCommand(WheelController_t* wheel_controller,
                                         float command);

/**
 * Function pushes a newly sampled output(wheel state) to the WheelController. The sampled
 * output is the current state of the wheel measured from sensors. This is in the same
 * units as the command.
 *
 * @param wheel_controller [in] The WheelController that will receive the new output
 * sample.
 *
 * @param output_sample Measured value of the current wheel state.
 */
void app_wheel_controller_pushNewSampleOutput(WheelController_t* wheel_controller,
                                              float output_sample);

/**
 * Function returns a new wheel voltage to apply to the wheel motor (V) based on previous
 * command input and previous sampled wheel states.
 *
 * Note: Positive voltages correspond to the 'forwards' direction, and negative voltages
 * to the 'negative' direction
 *
 * @param wheel_controller [in] The WheelController to calculate the new applied voltage
 * for.
 *
 * @return Wheel voltage to apply to achieve the desired state.
 */
float app_wheel_controller_getWheelVoltageToApply(WheelController_t* wheel_controller);

/**
 * Function destroys the specified instance of WheelController, de-allocating all memory
 * reserved for it.
 *
 * @param wheel_controller [in] WheelController_t to destroy.
 */
void app_wheel_controller_destroy(WheelController_t* wheel_controller);
