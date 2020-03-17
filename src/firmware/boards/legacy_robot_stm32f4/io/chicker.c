/**
 * \defgroup CHICKER Kicker and Chipper Functions
 *
 * These functions provide the ability to kick the ball.
 *
 * @{
 */
#include "chicker.h"

#include <FreeRTOS.h>
#include <minmax.h>
#include <rcc.h>
#include <registers/timer.h>
#include <task.h>

#include "adc.h"
#include "breakbeam.h"
#include "charger.h"
#include "firmware/shared/util.h"

/**
 *  \brief Defines max limits for kicking and chipping.
 */
#define MAX_KICK_VALUE 8.0f
#define MAX_CHIP_VALUE 2.0f

/**
 * \brief This is a hard cap on the pulse_width to prevent the fuse from blowing.
 */
#define KICKER_MAX_PULSE 10000U

/**
 * \brief The amount of time (in ticks) to delay after firing a device before allowing
 * another fire, to avoid physical collisions.
 */
#define COLLIDE_TIMEOUT (500U / portTICK_PERIOD_MS)

/**
 * \brief Whether or not autokick is armed.
 */
static bool auto_enabled = false;

/**
 * \brief Whether or not autokick has fired since the last successful feedback packet.
 */
static bool auto_fired = false;

/**
 * \brief Which device to fire when autokicking.
 */
static chicker_device_t auto_device;

/**
 * \brief The pulse width to use when autokicking.
 */
static unsigned int auto_width;

/**
 * \brief The number of ticks for which no device should be activated because
 * it would interfere with an ongoing fire.
 */
static unsigned int collide_timeout;

/**
 * \brief Whether or not discharge is requested.
 */
static bool chicker_discharge_requested = false;

/**
 * \brief Whether or not the hardware is currently generating the discharge
 * pulse train.
 */
static bool chicker_discharge_active = false;

/**
 * \brief robot index
 *
 */
static int index = 0;

/**
 * \brief Major scale duty cycle
 *
 */
static int CCR_val[8] = {91U / 2, 81U / 2, 73U / 2, 68U / 2,
                         61U / 2, 55U / 2, 48U / 2, 45U / 2};


/**
 * \brief Major scale freqency
 *
 */
static int ARR_val[8] = {2273U, 2020U, 1818U, 1705U, 1515U, 1364U, 1212U, 1136U};

/**
 * \brief Converts desired chicker power to a pulse width.
 */
unsigned int chicker_power_to_pulse_width(float power, bool chip)
{
    unsigned int width;
    if (!chip)
    {
        /*
         * Transfer function derived by measuring ball speed form various input
         * power levels form 2 m/s to 7 m/s and finding a linear relationship.
         *
         * Transfer function set for 2 cap chicker.
         */
        // limit(&power, MAX_KICK_VALUE);
        if (power > MAX_KICK_VALUE)
            power = MAX_KICK_VALUE;
        width = (unsigned int)(power * 438.1f + 44.592f);
    }
    else
    {
        // limit(&power, MAX_CHIP_VALUE);
        if (power > MAX_CHIP_VALUE)
            power = MAX_CHIP_VALUE;
        width = (unsigned int)(835.0f * power * power + 469.2f * power + 1118.5f);
    }

    // clamp(&width, 0.0f, (float) UINT16_MAX);
    return width;
}

/**
 * \brief Configures the kicker pulse generator to idle.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_kicker_idle(void)
{
    // Force output off.
    TIM10_14_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b100,  // Force OC1REF to low level
        }};
    TIM11.CCMR1 = ccmr1;
}

/**
 * \brief Configures the chipper pulse generator to idle.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_chipper_idle(void)
{
    // Force output off.
    TIM9_12_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b100,  // Force OC1REF to low level
        }};
    TIM9.CCMR1 = ccmr1;
}

/**
 * \brief Configures the kicker pulse generator to generate a single pulse.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_kicker_fire(unsigned int width)
{
    // Force output off while reconfiguring.
    chicker_hw_kicker_idle();

    // Disable the timer.
    TIM10_14_CR1_t cr1 = {
        .CEN = 0,
    };
    TIM11.CR1 = cr1;

    // Load the pulse width, clamping at maximum.
    TIM11.ARR  = MIN(width, KICKER_MAX_PULSE);
    TIM11.CCR1 = MIN(width, KICKER_MAX_PULSE);

    // Clear the counter.
    TIM11.CNT = 0U;

    // Begin the pulse by forcing the output high.
    TIM10_14_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b101,  // Force OC1REF to high level
        }};
    TIM11.CCMR1 = ccmr1;

    // Switch the timer to output-compare inactive-on-match mode.
    ccmr1.O.OC1M = 0b010U;
    TIM11.CCMR1  = ccmr1;

    // Start the counter.
    cr1.CEN   = 1;
    TIM11.CR1 = cr1;
}

/**
 * \brief Configures the chipper pulse generator to generate a single pulse.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_chipper_fire(unsigned int width)
{
    // Force output off while reconfiguring.
    chicker_hw_chipper_idle();

    // Disable the timer.
    TIM9_12_CR1_t cr1 = {
        .CEN = 0,
    };
    TIM9.CR1 = cr1;

    // Load the pulse width, clamping at maximum.
    TIM9.ARR  = MIN(width, 65535U);
    TIM9.CCR1 = MIN(width, 65535U);

    // Clear the counter.
    TIM9.CNT = 0U;

    // Begin the pulse by forcing the output high.
    TIM9_12_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b101,  // Force OC1REF to high level
        }};
    TIM9.CCMR1 = ccmr1;

    // Switch the timer to output-compare inactive-on-match mode.
    ccmr1.O.OC1M = 0b010U;
    TIM9.CCMR1   = ccmr1;

    // Start the counter.
    cr1.CEN  = 1;
    TIM9.CR1 = cr1;
}

/**
 * \brief Configures the kicker pulse generator to generate a discharge pulse
 * train.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_kicker_discharge(void)
{
    // Force output off while reconfiguring.
    chicker_hw_kicker_idle();

    // Disable the timer.
    TIM10_14_CR1_t cr1 = {
        .CEN = 0,
    };
    TIM11.CR1 = cr1;

    // Reconfigure timing parameters.
    TIM11.CNT = 0U;
    if (index > 0 && index < 8)
    {
        TIM11.ARR  = ARR_val[index] * 4U;
        TIM11.CCR1 = CCR_val[index] * 4U;
    }
    else
    {
        TIM11.ARR  = 1000U * 4U;
        TIM11.CCR1 = 25U * 4U;
    }

    // Enable the timer.
    cr1.CEN   = 1;
    TIM11.CR1 = cr1;

    // Enable the output.
    TIM10_14_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b110,  // PWM mode 1
        }};
    TIM11.CCMR1 = ccmr1;
}

/**
 * \brief Configures the chipper pulse generator to generate a discharge pulse
 * train.
 *
 * \pre This function must be called with appropriate protection against
 * simultaneous invocation.
 */
static void chicker_hw_chipper_discharge(void)
{
    // Force output off while reconfiguring.
    chicker_hw_chipper_idle();

    // Disable the timer.
    TIM9_12_CR1_t cr1 = {
        .CEN = 0,
    };
    TIM9.CR1 = cr1;

    // Reconfigure timing parameters.
    TIM9.CNT = 0U;
    if (index > 0 && index < 8)
    {
        TIM9.ARR  = ARR_val[index] * 4U;
        TIM9.CCR1 = CCR_val[index] * 4U;
    }
    else
    {
        TIM9.ARR  = 1000U * 4U;
        TIM9.CCR1 = 25U * 4U;
    }

    // Enable the timer.
    cr1.CEN  = 1;
    TIM9.CR1 = cr1;

    // Enable the output.
    TIM9_12_CCMR1_t ccmr1 = {
        .O = {
            .CC1S  = 0b00,  // Channel is an output
            .OC1FE = 0,     // No special fast enable circuit
            .OC1PE = 0,  // Writes to CCR1 happen immediately (we will be doing all that
                         // sort of thing with the timer disabled anyway)
            .OC1M = 0b110,  // PWM mode 1
        }};
    TIM9.CCMR1 = ccmr1;
}

/**
 * \brief Initializes the chicker subsystem.
 */
void chicker_init(unsigned int robot_index)
{
    // Configure timers 9 and 11 with a prescaler suitable to count microseconds.
    // The ideal way to handle these timers would be to enable one-pulse PWM mode.
    // Each firing could then set CCR to 1, set ARR to pulse width plus one, and enable
    // the counter. Unfortunately, timer 11 does not support one-pulse mode. So, instead,
    // we use output-compare mode along with some forced levels to get things into the
    // states we want.
    rcc_enable_reset(APB2, TIM9);
    rcc_enable_reset(APB2, TIM11);

    chicker_hw_kicker_idle();
    chicker_hw_chipper_idle();

    TIM9.PSC           = 168000000U / 1000000U;
    TIM9_12_EGR_t egr9 = {
        .UG = 1,  // Generate an update event (to push PSC into the timer)
    };
    TIM9.EGR             = egr9;
    TIM9_12_CCER_t ccer9 = {
        .CC1E  = 1,  // OC1 signal output to pin
        .CC1P  = 0,  // OC1 active high
        .CC1NP = 0,  // Must be cleared for output mode
    };
    TIM9.CCER = ccer9;

    TIM11.PSC            = 168000000U / 1000000U;
    TIM10_14_EGR_t egr11 = {
        .UG = 1,  // Generate an update event (to push PSC into the timer)
    };
    TIM11.EGR              = egr11;
    TIM10_14_CCER_t ccer11 = {
        .CC1E  = 1,  // OC1 signal output to pin
        .CC1P  = 0,  // OC1 active high
        .CC1NP = 0,  // Must be cleared for output mode
    };
    TIM11.CCER = ccer11;

    index = robot_index;
}

/**
 * \brief Shuts down the chicker subsystem.
 *
 * \post The safe discharge process has discharged the capacitors.
 */
void chicker_shutdown(void)
{
    chicker_discharge(true);
    collide_timeout = 0;
    chicker_tick();
    unsigned int timeout =
        30U;  // Give up if we wait three seconds and have not discharged (else ADC
              // failure could burn a lot of power forever).
    while (adc_capacitor() > CHICKER_DISCHARGE_THRESHOLD && timeout--)
    {
        vTaskDelay(100U / portTICK_PERIOD_MS);
    }
    chicker_discharge(false);
    chicker_tick();
}

/**
 * \brief Enables or disables the safe discharge pulse generator.
 *
 * \warning The safe discharge pulse generator should be disabled once the capacitors are
 * discharged!
 *
 * \param[in] discharge \c true to discharge, or \c false to not discharge
 */
void chicker_discharge(bool discharge)
{
    __atomic_store_n(&chicker_discharge_requested, discharge, __ATOMIC_RELAXED);
}

/**
 * \brief Fires a device with simultaneous access protection provided
 * externally.
 *
 * \param[in] device which device to fire
 * \param[in] width the width of the pulse, in kicking units
 */
static void chicker_fire_impl(chicker_device_t device, unsigned int width)
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
        chicker_hw_kicker_fire(width);
        chicker_hw_chipper_idle();
    }
    else
    {
        chicker_hw_kicker_idle();
        chicker_hw_chipper_fire(width);
    }

    // Set a timeout to avoid colliding with the hardware.
    collide_timeout = COLLIDE_TIMEOUT;

    // We are not currently discharging, because we have just taken control
    // of the hardware for a fire pulse.
    chicker_discharge_active = false;

    // Because we are discharging, the capacitors cannot be full.
    charger_mark_fired();
}

/**
 * \brief Fires a device.
 *
 * \param[in] device which device to fire
 * \param[in] width the width of the pulse, in kicking units
 */
void chicker_fire_with_pulsewidth(chicker_device_t device, unsigned int width)
{
    taskENTER_CRITICAL();
    chicker_fire_impl(device, width);
    taskEXIT_CRITICAL();
}

/**
 * \brief Fires a device with the given power
 * \param device The device to fire
 * \param power The power to fire with. For the kicker this is the speed, in meters per
 *              second. For the chipper this is the distance to the first bounce of the
 *              ball
 */
void chicker_fire_with_power(chicker_device_t device, float power)
{
    unsigned int width = chicker_power_to_pulse_width(power, device == CHICKER_CHIP);
    chicker_fire_with_pulsewidth(device, width);
}

/**
 * \brief Arms auto-fire mode.
 *
 * In auto-fire mode, the device is fired as soon as the break beam is interrupted.
 *
 * \param[in] device which device to fire
 * \param[in] width the width of the pulse, in kicking units
 */
void chicker_auto_arm(chicker_device_t device, float power)
{
    unsigned int width = chicker_power_to_pulse_width(power, device == CHICKER_CHIP);
    // The auto_device and auto_width variables are only read from an ISR, and only if
    // auto_enabled is true. Thus, auto_enabled being false can itself protect writes to
    // auto_device and auto_width, because ISRs are themselves implicitly atomic.
    __atomic_store_n(&auto_enabled, false, __ATOMIC_RELAXED);
    __atomic_signal_fence(__ATOMIC_ACQUIRE);
    auto_device = device;
    auto_width  = width;
    __atomic_signal_fence(__ATOMIC_RELEASE);
    __atomic_store_n(&auto_enabled, true, __ATOMIC_RELAXED);
}

/**
 * \brief Disarms auto-fire mode.
 *
 * Auto-fire mode is also automatically disarmed after it fires.
 */
void chicker_auto_disarm(void)
{
    __atomic_store_n(&auto_enabled, false, __ATOMIC_RELAXED);
}

/**
 * \brief Checks whether auto-fire mode is armed.
 *
 * \retval true auto-fire mode is armed
 * \retval false auto-fire mode is disarmed
 */
bool chicker_auto_armed(void)
{
    return __atomic_load_n(&auto_enabled, __ATOMIC_RELAXED);
}

/**
 * \brief Checks whether auto-fire has fired.
 *
 * \retval true auto-fire has fired since the last call to this function
 * \retval false auto-fire has not fired
 */
bool chicker_auto_fired_test_clear(void)
{
    const bool NEW_VALUE = false;
    bool ret;
    __atomic_exchange(&auto_fired, &NEW_VALUE, &ret, __ATOMIC_RELAXED);
    return ret;
}

/**
 * \brief Fires the kicker with force required to move the ball at the given speed
 * \param speed_m_per_s The speed that the ball would be kicked at, if it were right
 *                      in front of the kicker
 */
void chicker_kick(float speed_m_per_s)
{
    chicker_fire_with_power(CHICKER_KICK, speed_m_per_s);
}

/**
 * \brief Fires the chipper with force required to chip the ball the given distance
 * \param distance_m The distance to the first bounce the ball would be chipped, if it
 *                   were right in front of the chipper when the chipper fired
 */
void chicker_chip(float distance_m)
{
    chicker_fire_with_power(CHICKER_KICK, distance_m);
}

/**
 * \brief Enables autokick, so the kicker will fire if the ball comes in front of it
 * \param speed_m_per_s The speed to kick the ball at
 */
void chicker_enable_auto_kick(float speed_m_per_s)
{
    chicker_auto_arm(CHICKER_KICK, speed_m_per_s);
}

/**
 * \brief Enables autochip, so the chipper will fire if the ball comes in front of it
 * \param distance_m The distance to chip the ball (to the first bounce)
 */
void chicker_enable_auto_chip(float distance_m)
{
    chicker_auto_arm(CHICKER_CHIP, distance_m);
}

/**
 * \brief Counts down the timeout between consecutive fires.
 *
 * This function runs in the normal-speed tick task.
 */
void chicker_tick(void)
{
    // Check capacitor level for whether discharging would happen.
    bool above_discharge_threshold = adc_capacitor() > CHICKER_DISCHARGE_THRESHOLD;

    // Do this in a critical section so we cannot interleave with another task
    // or the fast tick ISR poking at the hardware and/or the collide timeout.
    taskENTER_CRITICAL();

    // Decrement the collide timeout.
    if (collide_timeout)
    {
        --collide_timeout;
    }

    // We want to discharge if we were asked to, no kick or chip has happened
    // recently, and the voltage is high enough.
    bool want_discharge =
        chicker_discharge_requested && !collide_timeout && above_discharge_threshold;

    // Update discharging state if necessary.
    if (want_discharge && !chicker_discharge_active)
    {
        chicker_hw_kicker_discharge();
        chicker_hw_chipper_discharge();
        chicker_discharge_active = true;
    }
    else if (!want_discharge && chicker_discharge_active)
    {
        chicker_hw_kicker_idle();
        chicker_hw_chipper_idle();
        chicker_discharge_active = false;
    }

    // Done critical section work.
    taskEXIT_CRITICAL();

    if (want_discharge)
    {
        // Because we are discharging, the capacitors cannot be full.
        charger_mark_fired();
    }
}

/**
 * \brief Checks for auto-fire eligibility.
 *
 * This function runs in the fast ISR.
 */
void chicker_tick_fast(void)
{
    // Check if we should fire now.
    if (auto_enabled && breakbeam_interrupted() && charger_full() && !collide_timeout)
    {
        chicker_fire_impl(auto_device, auto_width);
        auto_enabled = false;
        auto_fired   = true;
    }
}

/**
 * @}
 */
