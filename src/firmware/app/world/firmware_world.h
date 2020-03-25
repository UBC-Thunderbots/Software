#pragma once

#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"

/**
 * This struct represents the world from the perspective of the firmware
 */
typedef struct FirmwareWorld FirmwareWorld_t;

/**
 * Create a world
 *
 * @param robot The robot from the perspective of firmware
 * @param ball The ball from the perspective of firmware
 *
 * @return A pointer to the created world, ownership is given to the caller
 */
FirmwareWorld_t* app_firmware_world_create(FirmwareRobot_t* robot, FirmwareBall_t* ball);

/**
 * Destroy the given world, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param world The world to destroy
 */
void app_firmware_world_destroy(FirmwareWorld_t* world);

/**
 * Get the robot from the given world
 * @param world The world to get the robot from
 * @return The robot from the given world
 */
FirmwareRobot_t* app_firmware_world_getRobot(FirmwareWorld_t* world);

/**
 * Get the ball from the given world
 * @param world The world to get the ball from
 * @return The ball from the given world
 */
FirmwareBall_t* app_firmware_world_getBall(FirmwareWorld_t* world);
