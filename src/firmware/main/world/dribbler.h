#pragma once

#include <stdint.h>

/**
 * This struct represents a robot dribbler from the perspective of firmware
 */
struct Dribbler;
typedef struct Dribbler Dribbler;

/**
 * Create a dribbler with functions for interacting with it
 *
 * @param set_speed A function that can be called to set the current speed of the dribbler
 * @param get_temperature_deg_c A function that can be called to get the temperature of
 *                              the dribbler, in degrees celsius
 *
 * @return A pointer to the created dribbler, ownership is given to the caller
 */
Dribbler* Dribbler_create(void (*set_speed)(uint32_t rpm),
                          unsigned int (*get_temperature_deg_c)(void));

/**
 * Destroy the given dribbler, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `Dribbler_create`
 *
 * @param dribbler The dribbler to destroy
 */
void Dribbler_destroy(Dribbler* dribbler);

/**
 * Set the speed of the given dribbler
 * @param dribbler The dribbler to set the speed of
 * @param rpm The rpm to set the dribbler to
 */
void Dribbler_setSpeed(Dribbler* dribbler, uint32_t rpm);

/**
 * Get the temperature of the given dribbler
 * @param dribbler The dribbler to get the temperature of
 * @return The temperature of the dribbler, in degrees celsius
 */
unsigned int Dribbler_getTemperature(Dribbler* dribbler);
