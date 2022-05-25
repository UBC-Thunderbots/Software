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
 * @param wheel [in] The PrimitiveManager to destroy
 */
void app_primitive_manager_destroy(PrimitiveManager_t *manager);

/**
 * Start a new primitive with the given PrimitiveManager
 * @param manager [in/out] The PrimitiveManager to run the primitive
 * @param world [in] The world to run the primitive in
 * @param primitive_msg The message representing the primitive to run
 */
void app_primitive_manager_startNewPrimitive(PrimitiveManager_t *manager,
                                             FirmwareWorld_t *world,
                                             TbotsProto_Primitive primitive_msg);

/**
 * Runs the current primitive
 *
 * @param manager [in/out] The primitive manager to set the current primitive on
 * @param world [in] The world to run the primitive in
 */
void app_primitive_manager_runCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world);

/**
 * End the primitive the manager is currently running (if there is one running)
 * @param manager [in/out] The manager (that could be) running a primitive
 * @param world [in] The world the primitive is to be ended in
 */
void app_primitive_manager_endCurrentPrimitive(PrimitiveManager_t *manager,
                                               FirmwareWorld_t *world);
