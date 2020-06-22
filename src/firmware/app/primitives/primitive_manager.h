#pragma once

#include "firmware/app/primitives/primitive.h"
#include "firmware/app/world/firmware_world.h"

typedef struct PrimitiveManager PrimitiveManager_t;

/**
 * Create a PrimitiveManager
 * @return A PrimitiveManager
 */
PrimitiveManager_t *app_primitive_manager_create(void);

/**
 * Destroy the given PrimitiveManager, freeing any memory allocated for it
 *
 * @param wheel The PrimitiveManager to destroy
 */
void app_primitive_manager_destroy(PrimitiveManager_t *manager);

/**
 * Sets the current primitive to a new one
 *
 * @param manager The primitive manager to set the current primitive on
 * @param world The world to run the primitive in
 * @param primitive_index The index of the primitive to run
 * @param params The parameters for the primitive
 */
void app_primitive_manager_startNewPrimitive(PrimitiveManager_t *manager,
                                             FirmwareWorld_t *world,
                                             unsigned int primitive_index,
                                             const primitive_params_t *params);

/**
 * Runs the current primitive
 *
 * @param manager The primitive manager to set the current primitive on
 * @param world The world to run the primitive in
 */
void app_primitive_manager_runCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world);

/**
 * Gets the index of the currently running primitive
 * @param manager The primitive manager to get the index of the currently running
 *                primitive from
 * @return The index of the primitive this primitive manager is currently running, 254 if
 *         there is no primitive running
 */
unsigned int app_primitive_manager_getCurrentPrimitiveIndex(PrimitiveManager_t *manager);

/**
 * Checks whether a particular primitive is direct.
 *
 * @param primitive the primitive to check
 * @return true if the primitive is direct primitive, false otherwise
 */
bool app_primitive_manager_primitiveIsDirect(unsigned int primitive);
