#pragma once

#include "app/world/chicker.h"
#include "app/world/dribbler.h"
#include "app/world/wheel.h"

/**
 * This struct represents a robot from the perspective of firmware
 */
typedef struct FirmwareRobot FirmwareRobot_t;

/**
 * This struct represents robot constants
 */
typedef struct RobotConstants
{
    // TODO: jdocs for members
    float mass;
    float moment_of_inertia;
    float robot_radius;
    float jerk_limit;
} RobotConstants_t;

// TODO: this constructor needs to take functions for robot velocity
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
 * @param get_robot_orientation A function that can be called to get the orientation of
 *                              the robot, in radians
 * @param get_robot_velocity_x A function that can be called to get the x-velocity of the
 *                             robot, in m/s
 * @param get_robot_velocity_y A function that can be called to get the y-velocity of the
 *                             robot, in m/s
 * @param get_robot_velocity_angular A function that can be called to get the angular
 *                                   velocity of the robot, in rad/s
 * @param get_robot_acceleration_x A function that can be called to get the x-acceleration of the
 *                                 robot, in m/s^2
 * @param get_robot_acceleration_y A function that can be called to get the y-acceleration of the
 *                                 robot, in m/s^2
 * @param get_robot_acceleration_angular A function that can be called to get the angular
 *                                       acceleration of the robot, in rad/s^2
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
    float (*get_robot_position_y)(), float (*get_robot_orientation)(),
    float (*get_robot_velocity_x)(),
    float (*get_robot_velocity_y)(),
    float (*get_robot_velocity_angular)(),
    float (*get_robot_acceleration_x)(),
    float (*get_robot_acceleration_y)(),
    float (*get_robot_acceleration_angular)(),
    Wheel_t* front_right_wheel,
    Wheel_t* front_left_wheel, Wheel_t* back_right_wheel, Wheel_t* back_left_wheel,
    RobotConstants_t robot_constants);

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
 * Get the x-position of the given robot
 * @param robot The robot to get the y-position for
 * @return The x-position of the given robot, in meters, in global coordinates
 */
float app_firmware_robot_getPositionX(FirmwareRobot_t* robot);

/**
 * Get the y-position of the given robot
 * @param robot The robot to get the y-position for
 * @return The y-position of the given robot, in meters, in global coordinates
 */
float app_firmware_robot_getPositionY(FirmwareRobot_t* robot);

// TODO: testme
/**
 * Get the current orientation of the given robot
 *
 * @param robot The robot to get the orientation of
 * @return The orientation of the robot, in radians
 */
float app_firmware_robot_getOrientation(FirmwareRobot_t* robot);

// TODO: test me!
/**
 * Get the x-velocity of the given robot
 * @param robot The robot to get the y-velocity for
 * @return The x-velocity of the given robot, in m/s, in global coordinates
 */
float app_firmware_robot_getVelocityX(FirmwareRobot_t* robot);

// TODO: test me!
/**
 * Get the y-velocity of the given robot
 * @param robot The robot to get the y-velocity for
 * @return The y-velocity of the given robot, in m/s, in global coordinates
 */
float app_firmware_robot_getVelocityY(FirmwareRobot_t* robot);

/**
 * Get the current angular velocity for the given robot
 * @param robot The robot to get the angular velocity for
 * @return The angular velocity of the given robot, in rad/s
 */
float app_firmware_robot_getVelocityAngular(FirmwareRobot_t* robot);

// TODO: implement and test me
/**
 * Get the x-component of acceleration for the given robot
 * @param robot The robot to get the x-component of acceleration for
 * @return The x-component of acceleration for the given robot in m/s^2
 */
float app_firmware_robot_getAccelerationX(FirmwareRobot_t* robot);

// TODO: implement and test me
/**
 * Get the y-component of acceleration for the given robot
 * @param robot The robot to get the y-component of acceleration for
 * @return The y-component of acceleration for the given robot in m/s^2
 */
float app_firmware_robot_getAccelerationY(FirmwareRobot_t* robot);

// TODO: implement and test me
/**
 * Get the angular acceleration for the given robot
 * @param robot The robot to get the angular acceleration for
 * @return The angular acceleration for the given robot in rad/s^2
 */
float app_firmware_robot_getAccelerationAngular(FirmwareRobot_t* robot);

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

// TODO: test and implement me
/**
 * Get the battery voltage for the given robot
 * @param robot The robot to get the battery voltage for
 * @return The battery voltage for the given robot, in volts
 */
float app_firmware_robot_getBatteryVoltage(FirmwareRobot_t* robot);

/**
 * Get the physical constants for this robot
 * @param robot The robot to get physical constants from
 * @return The physical constants for the given robot
 */
// TODO: should this be "physical constants", or just "constants"
const RobotConstants_t app_firmware_robot_getPhysicalConstants(FirmwareRobot_t* robot);