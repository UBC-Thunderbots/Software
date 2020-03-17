/**
 * \defgroup ENCODER Optical Encoder Functions
 *
 * \brief These functions handle reading optical encoders to discover motor shaft speeds.
 *
 * @{
 */
#include "io/encoder.h"

#include <assert.h>
#include <math.h>
#include <rcc.h>
#include <registers/timer.h>

#include "io/hall.h"
#include "io/wheels.h"
#include "util/error.h"

#define NUM_ENCODERS 4U

static uint16_t last_positions[NUM_ENCODERS];
static int16_t speeds[NUM_ENCODERS];

/**
 * \brief Initializes the optical encoder speed measurement subsystem.
 */
void encoder_init(void)
{
    // Enable the encoder-reading timers.
    rcc_enable_reset(APB1, TIM2);
    rcc_enable_reset(APB1, TIM3);
    rcc_enable_reset(APB1, TIM4);
    rcc_enable_reset(APB1, TIM5);
    TIM2_5_SMCR_t smcr = {
        .SMS = 0b011,  // Encoder mode 3, count on both inputs
    };
    TIM2.SMCR            = smcr;
    TIM3.SMCR            = smcr;
    TIM4.SMCR            = smcr;
    TIM5.SMCR            = smcr;
    TIM2_5_CCMR1_t ccmr1 = {
        .I =
            {
                .CC1S = 0b01,  // CC1 is input, TI1 = IC1
                .CC2S = 0b01,  // CC2 is input, TI2 = IC2
            },
    };
    TIM2.CCMR1       = ccmr1;
    TIM3.CCMR1       = ccmr1;
    TIM4.CCMR1       = ccmr1;
    TIM5.CCMR1       = ccmr1;
    TIM2.ARR         = 65535U;
    TIM3.ARR         = 65535U;
    TIM4.ARR         = 65535U;
    TIM5.ARR         = 65535U;
    TIM2_5_CR1_t cr1 = {
        .CEN = 1,  // Enable counter
    };
    TIM2.CR1         = cr1;
    TIM3.CR1         = cr1;
    TIM4.CR1         = cr1;
    TIM5.CR1         = cr1;
    TIM2_5_EGR_t egr = {
        .UG = 1,  // Generate update event
    };
    TIM2.EGR = egr;
    TIM3.EGR = egr;
    TIM4.EGR = egr;
    TIM5.EGR = egr;

    // Take a reading from the timers to initialize the last-position record.
    encoder_tick();
}

/**
 * \brief Updates the current optical encoder speed measurements.
 */
void encoder_tick(void)
{
    // Grab new position data.
    uint16_t new_positions[NUM_ENCODERS];
    new_positions[0U] = (uint16_t)TIM2.CNT;
    new_positions[1U] = (uint16_t)TIM4.CNT;
    new_positions[2U] = (uint16_t)TIM3.CNT;
    new_positions[3U] = (uint16_t)TIM5.CNT;
    for (unsigned int i = 0U; i != NUM_ENCODERS; ++i)
    {
        speeds[i]         = -(int16_t)(uint16_t)(new_positions[i] - last_positions[i]);
        last_positions[i] = new_positions[i];
    }
}

/**
 * \brief Checks for encoder commutation errors.
 *
 * This can’t be done in @ref encoder_tick because it has to happen after
 * locking the Hall sensor readings.
 */
void encoder_check_commutation_errors(void)
{
    // Update encoder-not-commutating errors.
    for (unsigned int i = 0U; i != 4U; ++i)
    {
        float hall_rpt    = hall_speed(i) / (float)WHEELS_HALL_COUNTS_PER_REV;
        float encoder_rpt = encoder_speed(i) / (float)WHEELS_ENCODER_COUNTS_PER_REV;
        float diff        = fabsf(encoder_rpt - hall_rpt);
        // When Hall RPT is x, encoder RPT should be roughly x. However, the
        // extra WHEELS_ENCODER_COUNTS_PER_HALL_COUNT resolution means it is
        // not exactly x. That extra resolution is as much as
        // WHEELS_ENCODER_COUNTS_PER_HALL_COUNT encoder counts. In RPT, that is
        // (WHEELS_ENCODER_COUNTS_PER_HALL_COUNT /
        // WHEELS_ENCODER_COUNTS_PER_REV) RPT. Double that, and ignore any
        // errors with absolute magnitude less than that threshold.
        //
        // Also, to allow for physical imperfections in the sensors, also
        // ignore any error of less than 10%.
        //
        // Finally, do not consider the error to be *cleared* unless the wheel
        // is turning.
        if (diff > 2.0f * WHEELS_ENCODER_COUNTS_PER_HALL_COUNT /
                       WHEELS_ENCODER_COUNTS_PER_REV &&
            diff > 0.1f * fabsf(hall_rpt))
        {
            error_lt_set(ERROR_LT_ENC0_COMMUTATION + i, true);
        }
        else if (hall_speed(i))
        {
            error_lt_set(ERROR_LT_ENC0_COMMUTATION + i, false);
        }
    }
}

/**
 * \brief Reads the speed of a motor from its most recent optical encoder tick.
 *
 * \param[in] index the index of the optical encoder to read
 *
 * \return the speed of the shaft
 */
int16_t encoder_speed(unsigned int index)
{
    assert(index < NUM_ENCODERS);
    return speeds[index];
}

/**
 * @}
 */
