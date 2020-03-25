#include "io/lps.h"

#include <gpio.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "io/adc.h"
#include "io/pins.h"
#include "util/tbuf.h"
// LPS0 = PC0
// LPS1 = PC1
// LPS2 = PB4
// LPS3 = PB5

#define LPS_SENSOR_SPACING 2  // cm
//#define LPS_OFFSET 0.03f
#define LPS_MIN 0.001f
#define LPS_THRESH -0.15f
//#define LPS_ARRAY_SIZE 4
#define LPS_AVE_SAMPLE 20

// typedef float lps_values[LPS_ARRAY_SIZE];
typedef float lps_adc[16];

// calibration for offset and channel cross talks
static lps_values lps_offset;

// information for printing
static lps_values lps_raw;
static lps_values lps_offset = {
    0.197665f, 0.346932f, 0.345231f, 0.408780f
    // 0.0f, 0.0f, 0.0f, 0.0f
};
static lps_values lps_gain       = {0.129270f, 0.226474f, 0.205278f, 0.202057f};
static lps_values lps_moment_x   = {5.0251f, 11.6463f, 6.5249f, 1.5986f};
static lps_values lps_moment_y   = {3.5995f, 0.8201f, -1.8590f, -3.0401f};
static lps_values lps_abs_x      = {-3.9487f, -9.9363f, -5.6353f, -0.1425f};
static lps_values lps_abs_y      = {-2.7011f, 0.1604f, 0.7427f, 1.5515f};
static lps_values lps_variance_x = {-0.1220f, -0.4426f, -0.1617f, -0.1937f};
static lps_values lps_variance_y = {-0.1098f, -0.2124f, 0.2245f, 0.2258f};
static lps_values lps_norm;
static float lps_sum;
static float x, y;

// buffer to be accessed by isr and normal task
static unsigned int current_lps_buffer;
static lps_values lps_buf[3U];

static tbuf_t lps_buffer_ctl = TBUF_INIT;

void lps_init(void)
{
    current_lps_buffer = tbuf_write_get(&lps_buffer_ctl);
}

// math implemented
/* (4x1)      (4x16)               (16x4)
 * [LSB     [...(-1)^(n+1).....    [16x1
 *   .   =   ...(-1)^(n/2+1)...     ADC
 *   .       ...(-1)^(n/4+1)...  *  Values]
 *  MSB]     ...(-1)^(n/8+1)...]
 *
 */

// static lps_values* updating_lps;
// static lps_values* ready_lps_val;
// static lps_adc adc_values = {0,0,0,0 ,0,0,0,0 ,0,0,0,0 ,0,0,0,0};

void lps_incr(void)
{
    static unsigned int counter = 0;
    // static lps_values updating_lps = {0,0,0,0};
    float adc_reading = adc_lps();
    float* updating_lps;

    // adc_values[counter%16]=adc_reading;

    updating_lps = lps_buf[current_lps_buffer];
    for (unsigned int i = 0; i < LPS_ARRAY_SIZE; i++)
    {
        updating_lps[i] += ((counter & (1 << i)) ? 1 : -1) * adc_reading;
    }

    counter++;
    counter = counter % 16;

    if ((counter >> 0) & 1)
    {
        gpio_set(PIN_LPS_DRIVE3);
    }
    else
    {
        gpio_reset(PIN_LPS_DRIVE3);
    }
    if ((counter >> 1) & 1)
    {
        gpio_set(PIN_LPS_DRIVE2);
    }
    else
    {
        gpio_reset(PIN_LPS_DRIVE2);
    }
    if ((counter >> 2) & 1)
    {
        gpio_set(PIN_LPS_DRIVE1);
    }
    else
    {
        gpio_reset(PIN_LPS_DRIVE1);
    }
    if ((counter >> 3) & 1)
    {
        gpio_set(PIN_LPS_DRIVE0);
    }
    else
    {
        gpio_reset(PIN_LPS_DRIVE0);
    }


    if (counter == 0)
    {
        tbuf_write_put(&lps_buffer_ctl, current_lps_buffer);
        current_lps_buffer = tbuf_write_get(&lps_buffer_ctl);
        updating_lps       = lps_buf[current_lps_buffer];
        for (unsigned int i = 0; i < LPS_ARRAY_SIZE; i++)
        {
            updating_lps[i] = 0.0;
        }
    }
}

void lps_tick(void)
{
    // when the calculation is complete
    const float* updating_lps;
    unsigned int index;

    index        = tbuf_read_get(&lps_buffer_ctl);
    updating_lps = lps_buf[index];
    lps_sum      = 0.0f;

    for (unsigned int i = 0; i < LPS_ARRAY_SIZE; i++)
    {
        if (updating_lps[i] < LPS_THRESH)
        {
            lps_raw[i] = -updating_lps[i];
        }
        else
        {
            lps_raw[i] = LPS_MIN;
        }
    }
    x = 0.0f;
    y = 0.0f;
    for (unsigned int i = 0; i < LPS_ARRAY_SIZE; i++)
    {
        lps_norm[i] = (lps_raw[i] - lps_offset[i]) / lps_gain[i];
        // lps_mean += lps_norm[i] * (i-1.5f) * LPS_SENSOR_SPACING;
        // lps_var += lps_norm[i] * (i-1.5f)*(i-1.5f) * LPS_SENSOR_SPACING *
        // LPS_SENSOR_SPACING;
        x += lps_norm[i] * lps_moment_x[i] + (float)fabs(lps_norm[i]) * lps_abs_x[i] +
             lps_norm[i] * lps_norm[i] * lps_variance_x[i];
        y += lps_norm[i] * lps_moment_y[i] + (float)fabs(lps_norm[i]) * lps_abs_y[i] +
             lps_norm[i] * lps_norm[i] * lps_variance_y[i];
        lps_sum += lps_norm[i];
    }
    tbuf_read_put(&lps_buffer_ctl, index);
}

void lps_get_raw(lps_values lps_output)
{
    for (unsigned int i = 0; i < LPS_ARRAY_SIZE; i++)
    {
        lps_output[i] = lps_raw[i];
    }
}

void lps_get_pos(lps_values lps_output)
{
    lps_output[0] = x;
    lps_output[1] = y;
    lps_output[2] = lps_sum;
}
