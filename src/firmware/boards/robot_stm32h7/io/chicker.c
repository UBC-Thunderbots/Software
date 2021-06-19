#include "firmware/boards/robot_stm32h7/io/chicker.h"

#include "firmware/app/logger/logger.h"
#include "firmware/boards/robot_stm32h7/tim.h"

// Identify timers used for either KICK or CHIP
#define KICK_TIM &htim16
#define CHIP_TIM &htim17

// This is a hard cap on the pulse_width to minimize capacitor bank discharging
// This value is experimentally deduced from testing.
#define KICKER_MAX_PULSE 32000U

// The number of ticks for which no device should be activated because it would interfere
// with an ongoing fire.
//
#define COLLIDE_TIMEOUT 5000000U

// TODO (new): make sure there's no datarace on this variable
static unsigned int collide_timeout;

// The available devices.
typedef enum
{
    CHICKER_KICK,  ///< Performs a straight, flat kick.
    CHICKER_CHIP,  ///< Performs a chip kick up into the air.
} ChickerDevice_t;

/**
 * Fires chicker while checking for time collision
 *
 * @param device The device to chick with
 * @param width The pulse width
 */
static void chicker_fire(ChickerDevice_t device, unsigned int width);

// static void chipper_fire(unsigned int width);
static void kicker_fire(unsigned int width);
static unsigned int kicker_speedToPulseWidth(float speed_m_per_s);
static unsigned int chipper_distanceToPulseWidth(float distance_meters);
//static unsigned int kicker_timeToPulseWidth(float kick_pulse_time);
//static unsigned int chipper_timeToPulseWidth(float chip_pulse_time);

void io_chicker_tick()
{
    // TODO (new): call this from a task
    if (collide_timeout)
    {
        --collide_timeout;
    }
}

void io_chicker_kick(float speed_m_per_s)
{
    chicker_fire(CHICKER_KICK, kicker_speedToPulseWidth(speed_m_per_s));
}

void io_chicker_chip(float distance_m)
{
    chicker_fire(CHICKER_CHIP, chipper_distanceToPulseWidth(distance_m));
}

void io_chicker_enable_auto_kick(float speed_m_per_s)
{
    // TODO (#2080) implement this
}

void io_chicker_enable_auto_chip(float distance_m)
{
    // TODO (#2080) implement this
}

void io_chicker_disable_auto_kick()
{
    // TODO (#2080) implement this
}

void io_chicker_disable_auto_chip()
{
    // TODO (#2080) implement this
}

static void kicker_fire(unsigned int width)
{
    // TODO(new): test that this actually works

    TIM16->CCR1 = 1;
    TIM16->ARR = width < KICKER_MAX_PULSE ? width : KICKER_MAX_PULSE;
    HAL_TIM_OnePulse_Start(KICK_TIM, TIM_CHANNEL_1);
    // Trigger the timer directly in one pulse mode via the control register CR1
    // CEN : Counter Enable
    // OPM : One Pulse Mode
    TIM16->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;

    /* Testing for one pulse mode
    //HAL_TIM_OnePulse_DeInit(KICK_TIM);
    //HAL_TIM_OnePulse_Stop(KICK_TIM, TIM_CHANNEL_1);
    TIM16->CCR1 = width < KICKER_MAX_PULSE ? width : KICKER_MAX_PULSE;
    //HAL_TIM_OnePulse_Init(KICK_TIM, TIM_OPMODE_SINGLE);
    HAL_TIM_OnePulse_Start(KICK_TIM, TIM_CHANNEL_1);
    TIM16->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
    //HAL_TIM_OnePulse_Stop(KICK_TIM, TIM_CHANNEL_1);
    //HAL_TIM_OnePulse_DeInit(KICK_TIM);
    */
}

static void chipper_fire(unsigned int width)
{
    // TODO (new): implement this
}

static unsigned int kicker_speedToPulseWidth(float speed_m_per_s)
{
/*    // TODO (new): implement conversion from speed to pulse duration (time)
    float kick_time = 0.006f; //expect single digit ms range
    // TODO (new): programatically calculate pulse duration from RCC registers
    unsigned int pulse_width =
            __LL_TIM_CALC_PULSE(120000000,
                                __HAL_TIM_GET_ICPRESCALER(KICK_TIM, TIM_CHANNEL_1),

            )*/
    return 30000;
}

static unsigned int chipper_distanceToPulseWidth(float distance_meters)
{
    // TODO (new): implement this
    return 0;
}

static void chicker_fire(ChickerDevice_t device, unsigned int width)
{
    // A pulse width of zero is meaningless.
    if (!width)
    {
        return;
    }

    // We can only fire if we will not interfere with a previous fire.
    if (collide_timeout != 0)
    {
        return;
    }

    // Activate the hardware.
    if (device == CHICKER_KICK)
    {
        kicker_fire(width);
    }
    else
    {
        chipper_fire(width);
    }

    // Set a timeout to avoid colliding with the hardware.
    collide_timeout = COLLIDE_TIMEOUT;
}
