#pragma once

#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"

/**
 * This struct represents a robot from the perspective of firmware
 */
typedef struct FirmwareRobot FirmwareRobot_t;

/**
 * Create a robot with the given hardware
 *
 * NOTE: All positions are in global field coordinates (ie. 0,0 is the center of the
 *       field)
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
FirmwareRobot_t* app_firmware_robot_create(
    Chicker_t* chicker, Dribbler_t* dribbler, float (*get_robot_position_x)(),
    float (*get_robot_position_y)(), Wheel_t* front_right_wheel,
    Wheel_t* front_left_wheel, Wheel_t* back_right_wheel, Wheel_t* back_left_wheel);

/**
 * Destroy the given robot, freeing any memory allocated for it
 *
 * NOTE: This will not destroy the values pointed to by any pointers passed to the
 *       `create` function
 *
 * @param robot The robot to destroy
 */
void app_firmware_robot_destroy(FirmwareRobot_t* robot);

/**
 * Get the chicker from the given robot
 * @param robot The robot to get the chicker from
 * @return The chicker from the given robot
 */
Chicker_t* app_firmware_robot_getChicker(FirmwareRobot_t* robot);

/**
 * Get the dribbler from the given robot
 * @param robot The robot to get the dribbler from
 * @return The dribbler from the given robot
 */
Dribbler_t* app_firmware_robot_getDribbler(FirmwareRobot_t* robot);

/**
 * Get the x-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The x-position of the given ball, in meters, in global coordinates
 */
float app_firmware_robot_getPositionX(FirmwareRobot_t* robot);

/**
 * Get the y-position of the given ball
 * @param ball The ball to get the y-position for
 * @return The y-position of the given ball, in meters, in global coordinates
 */
float app_firmware_robot_getPositionY(FirmwareRobot_t* robot);

/**
 * Get the front right wheel from the given robot
 * @param robot The robot to get the front right wheel from
 * @return The front right wheel from the given robot
 */
Wheel_t* app_firmware_robot_getFrontRightWheel(FirmwareRobot_t* robot);

/**
 * Get the front left wheel from the given robot
 * @param robot The robot to get the front left wheel from
 * @return The front left wheel from the given robot
 */
Wheel_t* app_firmware_robot_getFrontLeftWheel(FirmwareRobot_t* robot);

/**
 * Get the back right wheel from the given robot
 * @param robot The robot to get the back right wheel from
 * @return The back right wheel from the given robot
 */
Wheel_t* app_firmware_robot_getBackRightWheel(FirmwareRobot_t* robot);


/**
 * Get the back left wheel from the given robot
 * @param robot The robot to get the back left wheel from
 * @return The back left wheel from the given robot
 */
Wheel_t* app_firmware_robot_getBackLeftWheel(FirmwareRobot_t* robot);
