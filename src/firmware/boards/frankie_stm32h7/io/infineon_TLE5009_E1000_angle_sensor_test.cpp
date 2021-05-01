extern "C"
{
#include "firmware/boards/frankie_stm32h7/io/infineon_TLE5009_E1000_angle_sensor.h"
}

#include <gtest/gtest.h>

TEST(InfineonTLE5009E1000AngleSensorTest, test_create_and_get_value)
{
    const float x_max_value             = 1.1f;
    const float x_min_value             = -1.0f;
    const float y_max_value             = 1.2f;
    const float y_min_value             = -1.3f;
    const float x_magnitude_45_degrees  = 0.6f;
    const float x_magnitude_135_degrees = 0.1f;
    const float y_magnitude_45_degrees  = 0.5f;
    const float y_magnitude_135_degrees = 0.8f;

    const float ave_x_value = (x_max_value + x_min_value) / 2;
    const float x_amplitude = (x_max_value - x_min_value) / 2;
    const float ave_y_value = (y_max_value + y_min_value) / 2;
    const float y_amplitude = (y_max_value - y_min_value) / 2;

    const float ave_45  = sqrtf(x_magnitude_45_degrees * x_magnitude_45_degrees +
                               y_magnitude_45_degrees * y_magnitude_45_degrees);
    const float ave_135 = sqrtf(x_magnitude_135_degrees * x_magnitude_135_degrees +
                                y_magnitude_135_degrees * y_magnitude_135_degrees);

    const float phase_offset_rad = 2 * atanf((ave_135 - ave_45) / (ave_135 + ave_45));

    float mock_x_data_sample = 0.66f;
    float mock_y_data_sample = -0.92f;

    InfineonTLE5009E1000AngleSensor_t* sensor = io_infineon_TLE5009_E1000_create(
        x_max_value, x_min_value, y_max_value, y_min_value, x_magnitude_45_degrees,
        y_magnitude_45_degrees, x_magnitude_135_degrees, y_magnitude_135_degrees);

    const float return_angle = io_infineon_TLE5009_E1000_calculateAngle(
        sensor, mock_x_data_sample, mock_y_data_sample);

    // Remove the mean voltage offset
    mock_x_data_sample -= ave_x_value;
    mock_y_data_sample -= ave_y_value;

    // Normalize gain
    mock_x_data_sample /= x_amplitude;
    mock_y_data_sample /= y_amplitude;

    // Phase offset correction
    mock_y_data_sample =
        (mock_y_data_sample - mock_x_data_sample * sinf(-phase_offset_rad)) /
        cosf(-phase_offset_rad);
    const float expected_angle = atan2f(mock_y_data_sample, mock_x_data_sample);

    EXPECT_FLOAT_EQ(expected_angle, return_angle);

    io_infineon_TLE5009_E1000_destroy(sensor);
}
