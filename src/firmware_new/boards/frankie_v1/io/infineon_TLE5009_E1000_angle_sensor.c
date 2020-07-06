#include "firmware_new/boards/frankie_v1/io/infineon_TLE5009_E1000_angle_sensor.h"

#include <stdlib.h>

struct InfineonTLE5009E1000AngleSensor
{
    // Gain/amplitude offset
    float A_x;
    float A_y;
    // Mean value offset
    float O_x;
    float O_y;
    // Phase offset
    float phase_difference_offset_rad;
};

InfineonTLE5009E1000AngleSensor_t* io_infineon_TLE5009_E1000_create(
    float x_max_value, float x_min_value, float y_max_value, float y_min_value,
    float x_magnitude_45_degrees, float y_magnitude_45_degrees,
    float x_magnitude_135_degrees, float y_magnitude_135_degrees)
{
    // This function intentionally uses variable names identical to the sensor data-sheet
    // Datasheet:
    // https://www.infineon.com/dgdl/Infineon-TLE5009_FDS-DataSheet-v01_01-en.pdf?fileId=db3a304330f686060131421d8ddd56b0
    // Calibration on page 19

    // Calculate the amplitudes
    const float A_x = (x_max_value - x_min_value) / 2;
    const float A_y = (y_max_value - y_min_value) / 2;

    // Calculate the mean voltage offset
    const float O_x = (x_max_value + x_min_value) / 2;
    const float O_y = (y_max_value + y_min_value) / 2;

    // Calculate the phase offset of the 2 waveforms
    const float M_45  = sqrtf(x_magnitude_45_degrees * x_magnitude_45_degrees +
                             y_magnitude_45_degrees * y_magnitude_45_degrees);
    const float M_135 = sqrtf(x_magnitude_135_degrees * x_magnitude_135_degrees +
                              y_magnitude_135_degrees * y_magnitude_135_degrees);
    const float phase_offset_rad = 2 * atanf((M_135 - M_45) / (M_135 + M_45));

    // Create and initialize struct
    InfineonTLE5009E1000AngleSensor_t* sensor =
        (InfineonTLE5009E1000AngleSensor_t*)malloc(
            sizeof(InfineonTLE5009E1000AngleSensor_t));
    sensor->A_x                         = A_x;
    sensor->A_y                         = A_y;
    sensor->O_x                         = O_x;
    sensor->O_y                         = O_y;
    sensor->phase_difference_offset_rad = phase_offset_rad;

    return sensor;
}

void io_infineon_TLE5009_E1000_destroy(InfineonTLE5009E1000AngleSensor_t* sensor)
{
    free(sensor);
}

float io_infineon_TLE5009_E1000_calculateAngle(InfineonTLE5009E1000AngleSensor_t* sensor,
                                               float x_value, float y_value)
{
    // Remove the mean voltage offset
    const float X_1 = x_value - sensor->O_x;
    const float Y_1 = y_value - sensor->O_y;

    // Normalize gain
    const float X_2 = X_1 / sensor->A_x;
    const float Y_2 = Y_1 / sensor->A_y;

    // Phase offset correction
    const float Y_3 = (Y_2 - X_2 * sinf(-sensor->phase_difference_offset_rad)) /
                      cosf(-sensor->phase_difference_offset_rad);

    return atan2f(Y_3, X_2);
}
