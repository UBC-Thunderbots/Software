#pragma once

#include "firmware/shared/circular_buffer.h"
#include "firmware/app/control/controller.h"

/**
 * Add value to input/output circular buffer
 *
 * @param buff The input/output circular buffer
 * @param coefficient The input/output coefficient from the controller
 */
void app_controller_addToBuffer(const CircularBuffer_t* buff, float coefficient)
{
    circular_buffer_push(buff, coefficient);
};

/**
 * Evaluate y_0
 *
 * @param xbuff The input circular buffer
 * @param ybuff The output circular buffer
 */
float app_controller_evaluate(const CircularBuffer_t* xbuff, const CircularBuffer_t* ybuff)
{
    // TODO take in the static coefficients
    // TODO make method to get array of values from xbuff and ybuff
}

/**
 * Clear input and output buffers
 *
 * @param xbuff The input circular buffer
 * @param ybuff The output circular buffer
 */
void app_controller_clear(const CircularBuffer_t* xbuff, const CircularBuffer_t* ybuff)
{
    circular_buffer_clear(xbuff);
    circular_buffer_clear(ybuff);
}
