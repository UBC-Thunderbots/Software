/** Initializes the system, and initially sets duty cycle to zero. If the
 * charger is enabed, the circut values are read and then the appropriate
 * registers are written to. Otherwise it sets the duty cycle to 0.
 **/

#include <FreeRTOS.h>
#include <gpio.h>
#include <rcc.h>
#include <registers/timer.h>
#include <stdbool.h>
#include <stdio.h>

#include "io/adc.h"
#include "io/receive.h"
#include "util/error.h"

#define INDUCTANCE 22e-6f
#define IMAX 10.0f
#define VDIODE 0.7f
#define CLOCK_PERIOD (1.0f / 168e6f)
#define PRESCALER 4

#define MAX_GATE_DRIVE_FREQUENCY 150e3f

#define CHARGE_TIMEOUT (4000U / portTICK_PERIOD_MS)

#define VC_MAX 240.0f

static bool charger_enabled         = false;
static bool full                    = false;
static unsigned int timeout_counter = CHARGE_TIMEOUT;

/**
 * \brief Calculates the MOSFET downtime.
 *
 * \param[in] vcap the cap voltage
 * \param[in] vbat the battery voltage
 * \return time in seconds
 */
static float downtime_calc(float vcap, float vbat)
{
    float time;
    if (vcap - vbat >= 1.0f)
    {
        time = /* Safety margin */ 1.10f * (IMAX * INDUCTANCE) / (vcap - vbat + VDIODE);
    }
    else
    {
        // Voltage difference too low.
        // vcap ~= vbat so the denominator approaches zero and the equation above falls
        // apart. Use a very conservative value instead.
        time = 0.002f;
    }
    if (time < 0.5f / MAX_GATE_DRIVE_FREQUENCY)
    {
        time = 0.5f / MAX_GATE_DRIVE_FREQUENCY;
    }
    return time;
}

/**
 * \brief Calculates the MOSFET uptime.
 *
 * \param[in] vbat the battery voltage
 * \return time in seconds
 */
static float uptime_calc(float vbat)
{
    return IMAX * INDUCTANCE / vbat * /* Safety margin */ 0.90f;
}

/**
 * \brief Calculates the number of ticks for the MOSFET uptime.
 *
 * \param[in] vbat the battery voltage
 * \return number of timer ticks
 */
static unsigned int capture_compare_register(float vbat)
{
    float upTime;
    unsigned int tickCount;
    upTime    = uptime_calc(vbat);
    tickCount = (unsigned int)(upTime / (CLOCK_PERIOD * PRESCALER));
    return tickCount;
}

/**
 * \brief Calculates the number of ticks for an entire cycle.
 *
 * \param[in] vcap the cap voltage
 * \param[in] vbat the battery voltage
 * \return number of timer ticks
 */
static unsigned int auto_reload_register(float vcap, float vbat)
{
    float totalTime;
    unsigned int tickCount;
    totalTime = downtime_calc(vcap, vbat) + uptime_calc(vbat);
    tickCount = (unsigned int)(totalTime / (CLOCK_PERIOD * PRESCALER));
    return tickCount;
}

/**
 * \brief Initializes the timer.
 */
void charger_init(void)
{
    rcc_enable_reset(APB2, TIM10);

    TIM10_14_CR1_t cr1 = {
        .CEN  = 1,  // enables counter
        .UDIS = 0,  // enables update events
        .URS  = 0,  // counter overflow and setting the UG bit cause UEV
        .ARPE = 1,  // TIM10_ARR registered is buffered
        .CKD  = 0,  // clock division set to 1
    };
    TIM10.CR1 = cr1;

    TIM10_14_CCMR1_output_t ccmr1 = {
        .CC1S  = 0,
        .OC1FE = 0,
        .OC1PE = 1,      // TIM10_CCR1 is preloaded and takes effect only at update event
        .OC1M  = 0b110,  // PWM mode 1, active as long as TIM10_CNT < TIM10_CCR1
    };
    TIM10.CCMR1.O = ccmr1;

    TIM10_14_CCER_t ccer = {
        .CC1E = 1,  // OC1 signal active
        .CC1P = 0,  // active = high
    };
    TIM10.CCER = ccer;

    uint32_t psc = PRESCALER - 1U;
    TIM10.PSC    = psc;
    TIM10.CNT    = 0;
}

/**
 * \brief Shuts down the charger.
 *
 * \post The charger is not charging.
 */
void charger_shutdown(void)
{
    // Set duty cycle to zero.
    TIM10.CCR1 = 0U;
}

/**
 * \brief Checks whether the capacitors are fully charged.
 *
 * \retval true the capacitors are full
 * \retval false the capacitors are below full charge
 */
bool charger_full(void)
{
    return __atomic_load_n(&full, __ATOMIC_RELAXED);
}

/**
 * \brief Marks the capacitors as having been used.
 *
 * This function must be called from anywhere that uses energy from the capacitors.
 */
void charger_mark_fired(void)
{
    __atomic_store_n(&full, false, __ATOMIC_RELAXED);  // Because energy has been used,
                                                       // capacitors are no longer full.
    __atomic_store_n(&timeout_counter, CHARGE_TIMEOUT,
                     __ATOMIC_RELAXED);  // Avoid reporting a timeout if we charge for a
                                         // long time because we keep firing.
}

/**
 * \brief Enables or disables the charger.
 *
 * \param[in] enabled whether or not to charge the capacitor
 */
void charger_enable(bool enabled)
{
    __atomic_store_n(&charger_enabled, enabled, __ATOMIC_RELAXED);
}

/**
 * \brief Sets the duty cycle to the correct value.
 */
void charger_tick(void)
{
    float vcap = adc_capacitor();
    float vbat = adc_battery_unfiltered();

    // printf("Cap Voltage: %f\n", vcap);

    bool charge = __atomic_load_n(&charger_enabled, __ATOMIC_RELAXED);

    if (!charge)
    {
        __atomic_store_n(&full, false,
                         __ATOMIC_RELAXED);  // If we are not charging at all, then we
                                             // instantly drain a bit.
    }

    if (vcap > VC_MAX)
    {
        charge = false;  // disable charger after critical voltage
        __atomic_store_n(&full, true, __ATOMIC_RELAXED);  // Charger is now full.
        __atomic_store_n(&timeout_counter, CHARGE_TIMEOUT,
                         __ATOMIC_RELAXED);  // We will keep topping up, but this does not
                                             // contribute to timeout.
    }

    if (vcap < (vbat - 0.7f) * 0.75f)
    {
        // This (vcap < vbat) should never happen.
        // Even with no pulses going out, the battery should drain at DC through the
        // inductor and diode into the capacitor. However, if the emergency relay were to
        // get stuck, the capacitors would be detached from the voltage divider. This
        // would result in vcap being roughly zero. Charging the capacitors in this
        // situation would be dangerous, because the discharge resistors would still be
        // attached!
        charge = false;
    }

    if (!timeout_counter /* Need not be atomic because only this task ever writes */)
    {
        // If timeout counter has reached zero, fail.
        error_lt_set(ERROR_LT_CHARGE_TIMEOUT, true);
    }

    if (error_lt_get(ERROR_LT_CHARGE_TIMEOUT) /* Need not be atomic because only this task ever writes */)
    {
        // If we have locked out due to timeout, fail.
        charge = false;
    }

    if (charge)
    {
        uint32_t ccrTicks;
        uint32_t arrTicks;

        ccrTicks = capture_compare_register(vbat);  // amount of ticks for MOSFET uptime
        arrTicks =
            auto_reload_register(vcap, vbat);  // total amount of ticks for one cycle
        TIM10_14_CR1_t cr1 = TIM10.CR1;
        cr1.UDIS           = 1;
        TIM10.CR1 = cr1;  // Disable update events while writing to ARR and CCR1 to ensure
                          // atomicity.
        TIM10.ARR  = arrTicks;  // assigns the length of total cycle
        TIM10.CCR1 = ccrTicks;  // assigns the length of MOSFET uptime
        cr1.UDIS   = 0;
        TIM10.CR1  = cr1;  // Re-enable update events.
        __atomic_fetch_sub(&timeout_counter, 1U,
                           __ATOMIC_RELAXED);  // Tick the timeout counter.
    }
    else
    {
        TIM10.CCR1 = 0;  // sets duty cycle to 0
        __atomic_store_n(&timeout_counter, CHARGE_TIMEOUT,
                         __ATOMIC_RELAXED);  // Reset timeout counter.
    }
}
