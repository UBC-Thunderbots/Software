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
 * @param chicker The robot chicker
 * @param dribbler The robot dribbler
 * @param get_robot_position_x A function that can be called to get the x-position of the
 *                             robot, in meters
 * @param get_robot_position_y A function that can be called to get the y-position of the
 *                             robot, in meters
 * @param front_right_wheel The front right wheel of the robot
 * @param front_left_wheel The front left wheel of the robot
 * @param back_right_wheel The back right wheel of the robot
 * @param back_left_wheel The back left wheel of the robot
 *
 * @return A pointer to a robot with the given hardware, ownership of the robot is
 *         given to the caller
 */
FirmwareRobot* FirmwareRobot_create(Chicker* chicker, Dribbler* dribbler,
                                    float (*get_robot_position_x)(),
                                    float (*get_robot_position_y)(),
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
 * Get the x-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The x-position of the given ball, in meters
 */
float FirmwareRobot_getPositionX(FirmwareRobot* robot);

/**
 * Get the y-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The y-position of the given ball, in meters
 */
float FirmwareRobot_getPositionY(FirmwareRobot* robot);

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
