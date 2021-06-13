#pragma once

#include <math.h>

#include "firmware/boards/robot_stm32h7/adc.h"
#include "firmware/boards/robot_stm32h7/tim.h"

/**
 * This file is an abstraction around the Infineon TLE5009 E1000 GMR angle sensor
 *
 * Datasheet:
 * https://www.infineon.com/dgdl/Infineon-TLE5009_FDS-DataSheet-v01_01-en.pdf?fileId=db3a304330f686060131421d8ddd56b0
 * Calibration on page 19.
 */

typedef struct InfineonTLE5009E1000AngleSensor InfineonTLE5009E1000AngleSensor_t;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);

/**
 * Creates an Infineon TLE5009 E1000 abstraction given the calibration parameters
 *
 * NOTE: The values here are the 'mean' values described in the data sheet. The mean is
 * equal to the summed average of the clock-wise and counter clock-wise calibration
 * values.
 *
 * @param x_max_value [in] The maximum value observed over a full period of the X output
 * waveform
 * @param x_min_value [in] The minimum value observed over a full period of the X output
 * waveform
 * @param y_max_value [in] The maximum value observed over a full period of the Y output
 * waveform
 * @param y_min_value [in] The minimum value observed over a full period of the Y output
 * waveform
 * @param x_magnitude_45_degrees [in] The X waveform magnitude at 45 degrees (as read by
 * the sensor)
 * @param y_magnitude_45_degrees [in] The Y waveform magnitude at 45 degrees (as read by
 * the sensor)
 * @param x_magnitude_135_degrees [in] The X waveform magnitude at 135 degrees (as read by
 * the sensor)
 * @param y_magnitude_135_degrees [in] The Y waveform magnitude at 135 degrees (as read by
 * the sensor)
 *
 * @return Infineon TLE5009E1000 abstraction
 */
InfineonTLE5009E1000AngleSensor_t* io_infineon_TLE5009_E1000_create(
    TIM_HandleTypeDef* timer, ADC_HandleTypeDef* adc, float x_max_value, float x_min_value, float y_max_value,
    float y_min_value, float x_magnitude_45_degrees, float y_magnitude_45_degrees,
    float x_magnitude_135_degrees, float y_magnitude_135_degrees);

/**
 * Destroys Infineon sensor abstraction
 *
 * @param sensor [in] The Infineon sensor abstraction to destroy
 */
void io_infineon_TLE5009_E1000_destroy(InfineonTLE5009E1000AngleSensor_t* sensor);

/**
 * Returns the calibration-corrected angle value from the input X and Y values
 *
 * @param sensor [in] The sensor abstraction
 * @param x_value [in] The sampled value of the X sinusoid
 * @param y_value [in] The sampled value of the Y sinusoid
 *
 * @return The calibration-corrected equivalent angle in radians
 */
float io_infineon_TLE5009_E1000_calculateAngle(InfineonTLE5009E1000AngleSensor_t* sensor,
                                               float x_value, float y_value);
