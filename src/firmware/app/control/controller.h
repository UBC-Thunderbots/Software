#pragma once

#include "firmware/shared/circular_buffer.h"

/**
 * Add value to output circular buffer
 *
 * @param ybuff The output circular buffer
 * @param output_coefficient The output coefficient y_# from the controller
 */
void app_controller_addOutput(const CircularBuffer_t* ybuff, float output_coefficient);

/**
 * Add value to input circular buffer
 *
 * @param xbuff The input circular buffer
 * @param input_coefficient The input coefficient x_# from the controller
 */
void app_controller_addInput(const CircularBuffer_t* xbuff, float input_coefficient);

/**
 * Evaluate y_0
 *
 * @param xbuff The input circular buffer
 * @param ybuff The output circular buffer
 */
void app_controller_evaluate(const CircularBuffer_t* xbuff, const CircularBuffer_t* ybuff);

/**
 * Clear input and output buffers
 *
 * @param xbuff The input circular buffer
 * @param ybuff The output circular buffer
 */
void app_controller_clear(const CircularBuffer_t* xbuff, const CircularBuffer_t* ybuff);
