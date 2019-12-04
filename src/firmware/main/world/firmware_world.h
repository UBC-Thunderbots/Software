#pragma once

#include "world/firmware_robot.h"

/**
 * This struct represents the world from the perspective of the firmware
 */
struct FirmwareWorld;
typedef struct FirmwareWorld FirmwareWorld;

/**
 * Create a world
 *
 * @param robot The robot from the perspective of firmware
 *
 * @return A pointer to the created world, ownership is given to the caller
 */
FirmwareWorld* FirmwareWorld_create(FirmwareRobot* robot);

/**
 * Destroy the given world, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `FirmwareWorld_create`
 *
 * @param world The world to destroy
 */
void FirmwareWorld_destroy(FirmwareWorld* world);

/**
 * Get the robot from the given world
 * @param world The world to get the robot from
 * @return The robot from the given world
 */
FirmwareRobot* FirmwareWorld_getRobot(FirmwareWorld* world);

