#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>

/**
 * \file
 *
 * \brief Handles the alert buzzer.
 */

/**
 * \brief Initializes the buzzer system.
 */
void buzzer_init(void);

/**
 * \brief Starts the buzzer sounding.
 *
 * The buzzer starts sounding when this function is called.
 * The buzzer stops when \ref buzzer_stop is called or when all requested time periods
 * have expired. Calling this function while the buzzer is already sounding extends the
 * remaining time the buzzer sounds, if necessary, to satisfy the request.
 *
 * \param millis the number of milliseconds to buzz for, starting from the current time
 */
void buzzer_start(unsigned long millis);

/**
 * \brief Stops the buzzer immediately.
 */
void buzzer_stop(void);

#endif
