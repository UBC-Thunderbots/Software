/**
 * \defgroup BB Break Beam Functions
 *
 * \brief These functions handle the break beam.
 *
 * @{
 */

#include "breakbeam.h"

#include <FreeRTOS.h>
#include <gpio.h>

#include "adc.h"
#include "feedback.h"
#include "pins.h"

#define THRESHOLD 0.01f
static bool laser_active = false, interrupted = false;
static float off_reading = 0.0f, on_reading = 0.0f;

/**
 * \brief Returns the light-to-dark voltage difference.
 *
 * \return the voltage difference
 */
float breakbeam_difference(void)
{
    // There are no atomic operations for floats, so disable interrupts while reading the
    // variables.
    portDISABLE_INTERRUPTS();
    float off = off_reading, on = on_reading;
    portENABLE_INTERRUPTS();
    return off - on;
}

/**
 * \brief Checks if the beam is interrupted.
 *
 * \retval true the beam is interrupted
 * \retval false the beam is clear
 */
bool breakbeam_interrupted(void)
{
    return __atomic_load_n(&interrupted, __ATOMIC_RELAXED);
}

/**
 * \brief Checks whether to pend a has-ball notification message.
 *
 * \param[out] log The log record to fill, or NULL if none.
 */
void breakbeam_tick(log_record_t *log)
{
    static bool old = false;
    bool new        = breakbeam_interrupted();
    if (new != old)
    {
        old = new;
        feedback_pend_has_ball();
    }
    if (log)
    {
        log->tick.breakbeam_diff = breakbeam_difference();
    }
}

/**
 * \brief Updates the state of the break beam.
 *
 * This function runs in the fast ISR.
 */
void breakbeam_tick_fast(void)
{
    float reading = adc_breakbeam();
    if (laser_active)
    {
        on_reading   = reading;
        laser_active = false;
        gpio_reset(PIN_BREAKBEAM_LASER);
    }
    else
    {
        off_reading  = reading;
        laser_active = true;
        gpio_set(PIN_BREAKBEAM_LASER);
    }
    interrupted = off_reading - on_reading < THRESHOLD;
}

/**
 * @}
 */
