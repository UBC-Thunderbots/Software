#include "firmware/boards/robot_stm32h7/io/infineon_TLE5009_E1000_angle_sensor.h"

#include <stdlib.h>
#include <math.h>

#include "firmware/app/logger/logger.h"

#define ADC_DMA_BUFFER __attribute__((section(".dma_buffer")))
#define RX_BUFFER_LENGTH_BYTES 18
#define NUM_ENCODERS 4

#define BACK_LEFT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define FRONT_RIGHT 3

ADC_DMA_BUFFER static uint16_t g_dma_adc_receive_buffer[RX_BUFFER_LENGTH_BYTES];
static ADC_HandleTypeDef* g_adc_handle;

volatile float past_speeds[NUM_ENCODERS] = {0};
volatile int16_t delta_speed[NUM_ENCODERS] = {0};
volatile uint32_t last_sampled_tick_time = 0u;

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

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t time_elapsed = 
    SCB_InvalidateDCache_by_Addr(
        (uint32_t*)(((uint32_t)g_dma_adc_receive_buffer) & ~(uint32_t)0x1F),
        RX_BUFFER_LENGTH_BYTES + 32);

    // TODO hack remove the VREF sampling later
    g_dma_adc_receive_buffer[8] = 32768;

    float front_right = atan2f(
            (float)(g_dma_adc_receive_buffer[1] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[0] - g_dma_adc_receive_buffer[8])
    );

    float back_right = atan2f(
            (float)(g_dma_adc_receive_buffer[5] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[2] - g_dma_adc_receive_buffer[8])
    );

    float front_left = atan2f(
            (float)(g_dma_adc_receive_buffer[6] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[3] - g_dma_adc_receive_buffer[8])
    );

    float back_left = atan2f(
            (float)(g_dma_adc_receive_buffer[7] - g_dma_adc_receive_buffer[8]),
            (float)(g_dma_adc_receive_buffer[4] - g_dma_adc_receive_buffer[8])
    );

    delta_speed[FRONT_RIGHT] = past_speeds[FRONT_RIGHT] - front_right;
    delta_speed[BACK_RIGHT] = past_speeds[BACK_RIGHT] - back_right;
    delta_speed[FRONT_LEFT] = past_speeds[FRONT_LEFT] - front_left;
    delta_speed[BACK_LEFT] = past_speeds[BACK_LEFT] - back_left;

    past_speeds[FRONT_RIGHT] = front_right;
    past_speeds[BACK_RIGHT] = back_right;
    past_speeds[FRONT_LEFT] = front_left;
    past_speeds[BACK_LEFT] = back_left;

    /*int front_right_int = (int)(front_right * 100);*/
    /*int front_left_int = (int)(front_left * 100);*/
    /*int back_right_int = (int)(back_right * 100);*/
    /*int back_left_int = (int)(back_left * 100);*/

    /*TLOG_INFO("Radians x 100: FR: %d, FL: %d, BR: %d, BL: %d", front_right_int, front_left_int, back_right_int, back_left_int);*/
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {}

InfineonTLE5009E1000AngleSensor_t* io_infineon_TLE5009_E1000_create(
    ADC_HandleTypeDef* adc, float x_max_value, float x_min_value, float y_max_value,
    float y_min_value, float x_magnitude_45_degrees, float y_magnitude_45_degrees,
    float x_magnitude_135_degrees, float y_magnitude_135_degrees)
{
    g_adc_handle = adc;

    if (HAL_ADC_Init(g_adc_handle) != HAL_OK)
    {
        TLOG_FATAL("Could not init adc");
    }

    if (HAL_ADC_Start_DMA(g_adc_handle, (uint32_t*)g_dma_adc_receive_buffer,
                          RX_BUFFER_LENGTH_BYTES) != HAL_OK)
    {
        TLOG_FATAL("Could not configure ADC DMA");
    }
    else
    {
        TLOG_INFO("Started ADC DMA in circular mode");
    }

    // This function intentionally uses variable names identical to the sensor data-sheet
    // Datasheet:
    // https://www.infineon.com/dgdl/Infineon-TLE5009_FDS-DataSheet-v01_01-en.pdf?fileId=db3a304330f686060131421d8ddd56b0
    // Calibration on page 19

    /*// Calculate the amplitudes*/
    /*const float A_x = (x_max_value - x_min_value) / 2;*/
    /*const float A_y = (y_max_value - y_min_value) / 2;*/

    /*// Calculate the mean voltage offset*/
    /*const float O_x = (x_max_value + x_min_value) / 2;*/
    /*const float O_y = (y_max_value + y_min_value) / 2;*/

    /*// Calculate the phase offset of the 2 waveforms*/
    /*const float M_45  = sqrtf(x_magnitude_45_degrees * x_magnitude_45_degrees +*/
    /*y_magnitude_45_degrees * y_magnitude_45_degrees);*/
    /*const float M_135 = sqrtf(x_magnitude_135_degrees * x_magnitude_135_degrees +*/
    /*y_magnitude_135_degrees * y_magnitude_135_degrees);*/
    /*const float phase_offset_rad = 2 * atanf((M_135 - M_45) / (M_135 + M_45));*/

    // Create and initialize struct
    InfineonTLE5009E1000AngleSensor_t* sensor = NULL;
    /*(InfineonTLE5009E1000AngleSensor_t*)malloc(*/
    /*sizeof(InfineonTLE5009E1000AngleSensor_t));*/
    /*sensor->A_x                         = A_x;*/
    /*sensor->A_y                         = A_y;*/
    /*sensor->O_x                         = O_x;*/
    /*sensor->O_y                         = O_y;*/
    /*sensor->phase_difference_offset_rad = phase_offset_rad;*/

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
