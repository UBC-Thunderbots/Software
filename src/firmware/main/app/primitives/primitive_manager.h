#pragma once

#include "firmware/main/app/primitives/primitive.h"
#include "firmware/main/app/world/firmware_world.h"

typedef struct PrimitiveManager PrimitiveManager_t;

/**
 * Create a PrimitiveManager
 * @return A PrimitiveManager
 */
PrimitiveManager_t* app_primitive_manager_create(void);

/**
 * Sets the current primitive to a new one
 *
 * @param manager The primitive manager to set the current primitive on
 * @param world The world to run the primitive in
 * @param primitive_index The index of the primitive to run
 * @param params The parameters for the primitive
 */
void app_primitive_manager_start_new_primitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world,
                                               unsigned int primitive_index,
                                               const primitive_params_t *params);

/**
 * Runs the current primitive
 *
 * @param manager The primitive manager to set the current primitive on
 * @param world The world to run the primitive in
 * @return ?? TODO ??
 */
void app_primitive_manager_run_current_primitive(PrimitiveManager_t *manager,
                                                    FirmwareWorld_t *world);

// TODO: does this belong here
// TODO: new jdoc style
/**
 * Checks whether a particular primitive is direct.
 *
 * @param primitive the primitive to check
 * @return true if the primitive is direct primitive, false otherwise
 */
bool app_primitive_manager_primitiveIsDirect(unsigned int primitive);

