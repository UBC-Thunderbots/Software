#ifndef ESTOP_H
#define ESTOP_H

/**
 * \file
 *
 * \brief Reads the emergency stop switch.
 */

#include <FreeRTOS.h>
#include <semphr.h>

/**
 * \brief The states the switch can be in.
 */
typedef enum
{
    /**
     * \brief The switch is faulty or not plugged in.
     */
    ESTOP_BROKEN,

    /**
     * \brief The switch is set to the stop position.
     */
    ESTOP_STOP,

    /**
     * \brief The switch is set to the run position.
     */
    ESTOP_RUN,
} estop_t;

/**
 * \brief Starts reading the emergency stop switch.
 *
 * This function is intended to be called once at application startup.
 * The emergency stop switch will be read continuously from that point forward on a timer.
 */
void estop_init(void);

/**
 * \brief Returns the most recently sampled state of the switch.
 *
 * \return the state of the switch at the most recent sample
 */
estop_t estop_read(void);

/**
 * \brief Sets the semaphore that will be notified when the switch changes state.
 *
 * \param[in] sem the semaphore to give
 */
void estop_set_sem(SemaphoreHandle_t sem);

/**
 * \brief Handles ADC interrupts.
 *
 * This function should be registered in the interrupt vector table at position 18.
 */
void adc_isr(void);

#endif
