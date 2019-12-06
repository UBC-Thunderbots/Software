#pragma once

#include "app/world/firmware_ball.h"
#include "app/world/firmware_robot.h"

/**
 * This struct represents the world from the perspective of the firmware
 */
struct FirmwareWorld;
typedef struct FirmwareWorld FirmwareWorld;

/**
 * Create a world
 *
 * @param robot The robot from the perspective of firmware
 * @param ball The ball from the perspective of firmware
 *
 * @return A pointer to the created world, ownership is given to the caller
 */
FirmwareWorld* app_firmware_world_create(FirmwareRobot* robot, FirmwareBall* ball);

/**
 * Destroy the given world, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param world The world to destroy
 */
void app_firmware_world_destroy(FirmwareWorld* world);

/**
 * Get the robot from the given world
 * @param world The world to get the robot from
 * @return The robot from the given world
 */
FirmwareRobot* app_firmware_world_getRobot(FirmwareWorld* world);

/**
 * Get the ball from the given world
 * @param world The world to get the ball from
 * @return The ball from the given world
 */
FirmwareBall* app_firmware_world_getBall(FirmwareWorld* world);
