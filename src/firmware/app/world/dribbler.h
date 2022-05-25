#pragma once

#include <stdint.h>

/**
 * This struct represents a robot dribbler from the perspective of firmware
 */
typedef struct Dribbler Dribbler_t;

/**
 * Create a dribbler with functions for interacting with it
 *
 * @param set_speed A function that can be called to set the current speed of the dribbler
 * @param coast A function that can be called to make the dribbler coast until another
 *              operation is applied to it
 * @param get_temperature_deg_c A function that can be called to get the temperature of
 *                              the dribbler, in degrees celsius
 *
 * @return A pointer to the created dribbler, ownership is given to the caller
 */
Dribbler_t* app_dribbler_create(void (*set_speed)(uint32_t rpm), void (*coast)(void),
                                unsigned int (*get_temperature_deg_c)(void));

/**
 * Destroy the given dribbler, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param dribbler The dribbler to destroy
 */
void app_dribbler_destroy(Dribbler_t* dribbler);

/**
 * Set the speed of the given dribbler
 *
 * @param dribbler The dribbler to set the speed of
 * @param rpm The rpm to set the dribbler to
 */
void app_dribbler_setSpeed(Dribbler_t* dribbler, uint32_t rpm);

/**
 * Sets the given dribbler to coast until another operation is applied to it
 *
 * @param dribbler The dribbler to coast
 */
void app_dribbler_coast(Dribbler_t* dribbler);

/**
 * Get the temperature of the given dribbler
 *
 * @param dribbler The dribbler to get the temperature of
 *
 * @return The temperature of the dribbler, in degrees celsius
 */
unsigned int app_dribbler_getTemperature(Dribbler_t* dribbler);
