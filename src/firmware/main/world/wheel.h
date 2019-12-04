#pragma once

/**
 * This struct represents a robot wheel, from the perspective of firmware
 */
struct Wheel;
typedef struct Wheel Wheel;

/**
 * Create a wheel object with functions for interacting with it
 * @param apply_wheel_force A function that we can call to apply a force to this wheel,
 *                          in newtons
 *
 * @return A pointer to the created wheel, ownership is given to the caller
 */
Wheel* Wheel_create(void (*apply_wheel_force)(float force_in_newtons));

/**
 * Destroy the given wheel, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `Wheel_create`
 *
 * @param wheel The wheel to destroy
 */
void Wheel_destroy(Wheel* wheel);

/**
 * Apply the given force to the given wheel
 * @param wheel The wheel to apply force to
 * @param force_in_newtons The force to apply to the wheel, in newtons
 */
void Wheel_applyForce(Wheel* wheel, float force_in_newtons);

