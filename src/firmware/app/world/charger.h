#pragma once

/**
 * This struct represents a robot capacitor charger from the perspective of firmware
 */
typedef struct Charger Charger_t;

/**
 * Create a charger with the given functions for interacting with it
 *
 * @param charge_capacitor Charge the capacitor of the robot
 * @param discharge_capacitor Discharge the capacitor of the robot
 * @param float_capacitor Set capacitor to float
 *
 * @return A pointer to the created charger, ownership is given to the caller
 */
Charger_t* app_charger_create(void (*charge_capacitor)(void),
                              void (*discharge_capacitor)(void),
                              void (*float_capacitor)(void));

/**
 * Destroy the given charger, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param charger The charger to destroy
 */
void app_charger_destroy(Charger_t* charger);

/**
 * Charge the capacitor on the given charger
 *
 * @param charger The charger
 */
void app_charger_charge_capacitor(Charger_t* charger);

/**
 * Discharge the capacitor on the given charger
 *
 * @param charger The charger
 */
void app_charger_discharge_capacitor(Charger_t* charger);

/**
 * Set capacitor to float on the given charger
 *
 * @param charger The charger
 */
void app_charger_float_capacitor(Charger_t* charger);
