#pragma once

#include "firmware/main/app/world/firmware_world.h"

// TODO: delete if unused

/**
 * This struct represents the state of the application layer of firmware
 */
typedef struct FirmwareAppState FirmwareAppState_t;

// TODO: impl me!
// TODO: jdocs

void* getCurrentPrimitiveState(FirmwareAppState_t* app_state);

FirmwareWorld_t* getFirmwareWorld(FirmwareAppState_t* app_state);

