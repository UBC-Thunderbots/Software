#pragma once

#include "world/chicker.h"
#include "world/dribbler.h"
#include "world/wheel.h"

/**
 * This struct represents a robot from the perspective of firmware
 */
struct FirmwareRobot;
typedef struct FirmwareRobot FirmwareRobot;

/**
 * Create a robot with the given hardware
 *
 * @param chicker
 * @param dribbler
 * @param front_right_wheel
 * @param front_left_wheel
 * @param back_right_wheel
 * @param back_left_wheel
 *
 * @return A pointer to a robot with the given hardware, ownership of the robot is
 *         given to the caller
 */
FirmwareRobot* FirmwareRobot_create(Chicker* chicker, Dribbler* dribbler,
                                    Wheel* front_right_wheel, Wheel* front_left_wheel,
                                    Wheel* back_right_wheel, Wheel* back_left_wheel);

/**
 * Destroy the given robot, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed into
 *       `FirmwareRobot_create`
 *
 * @param robot The robot to destroy
 */
void FirmwareRobot_destroy(FirmwareRobot* robot);

/**
 * Get the chicker from the given robot
 * @param robot The robot to get the chicker from
 * @return The chicker from the given robot
 */
Chicker* FirmwareRobot_getChicker(FirmwareRobot* robot);

/**
 * Get the dribbler from the given robot
 * @param robot The robot to get the dribbler from
 * @return The dribbler from the given robot
 */
Dribbler* FirmwareRobot_getDribbler(FirmwareRobot* robot);

/**
 * Get the front right wheel from the given robot
 * @param robot The robot to get the front right wheel from
 * @return The front right wheel from the given robot
 */
Wheel* FirmwareRobot_getFrontRightWheel(FirmwareRobot* robot);

/**
 * Get the front left wheel from the given robot
 * @param robot The robot to get the front left wheel from
 * @return The front left wheel from the given robot
 */
Wheel* FirmwareRobot_getFrontLeftWheel(FirmwareRobot* robot);

/**
 * Get the back right wheel from the given robot
 * @param robot The robot to get the back right wheel from
 * @return The back right wheel from the given robot
 */
Wheel* FirmwareRobot_getBackRightWheel(FirmwareRobot* robot);


/**
 * Get the back left wheel from the given robot
 * @param robot The robot to get the back left wheel from
 * @return The back left wheel from the given robot
 */
Wheel* FirmwareRobot_getBackLeftWheel(FirmwareRobot* robot);
