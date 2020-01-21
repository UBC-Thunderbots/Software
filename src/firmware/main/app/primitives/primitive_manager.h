#pragma once

#include "firmware/main/app/world/firmware_world.h"

// TODO: convert to new jdoc style

/**
 * \brief Initializes the movement primitive manager and all the primitives.
 */
void primitive_init(void);

/**
 * \brief Starts a new movement.
 *
 * \param[in] primitive the index of the primitive to run
 * \param[in] params the parameters to the primitive
 * \param[in] world The world to perform the primitive in
 */
void primitive_start(unsigned int primitive, const primitive_params_t *params,
                     FirmwareWorld_t *world);

// TODO: jdoc
uint8_t primitive_tick(FirmwareWorld_t *world);

/**
 * \brief Checks whether a particular primitive is direct.
 *
 * \param[in] primitive the primitive to check
 * \retval true the primitive is direct
 * \retval false the primitive is a movement primitive
 */
bool primitive_is_direct(unsigned int primitive);

// TODO: jdoc
unsigned int get_primitive_index();

// TODO: jdoc
bool primitive_params_are_equal(primitive_params_t *params1, primitive_params_t *params);
