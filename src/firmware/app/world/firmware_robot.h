#pragma once

#include "firmware/app/control/trajectory_planner.h"
#include "firmware/app/control/control.h"
#include "firmware/app/control/physbot.h"
#include "firmware/app/world/charger.h"
#include "firmware/app/world/chicker.h"
#include "firmware/app/world/dribbler.h"
#include "firmware/app/world/force_wheel.h"
#include "firmware/app/world/velocity_wheel.h"
#include "shared/proto/primitive.nanopb.h"
#include "shared/constants.h"
#include "firmware/shared/physics.h"
#include "firmware/shared/util.h"

/**
 * This struct represents a robot from the perspective of firmware
 */
typedef struct FirmwareRobot FirmwareRobot_t;

/**
 * Create a robot with the given hardware
 *
 * NOTE: Everything here is in the global field reference frame (ie. 0,0 is the center of
 *       the field, 0 degrees is towards the enemy goal) unless otherwise specified.
 *
 * @param charger The robot charger
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
 * @param get_battery_voltage A function that can be called to the batter voltage, in
 *                            volts
 * @param front_right_wheel The front right wheel of the robot
 * @param front_left_wheel The front left wheel of the robot
 * @param back_right_wheel The back right wheel of the robot
 * @param back_left_wheel The back left wheel of the robot
 * @param controller_state The controller state
 * @param robot_constants Set of constants particular to this robot
 *
 * @return A pointer to a robot with the given hardware, ownership of the robot is
 *         given to the caller
 */
FirmwareRobot_t* app_firmware_velocity_wheels_robot_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void), VelocityWheel_t* front_right_wheel,
    VelocityWheel_t* front_left_wheel, VelocityWheel_t* back_right_wheel,
    VelocityWheel_t* back_left_wheel, ControllerState_t* controller_state,
    RobotConstants_t robot_constants);

FirmwareRobot_t* app_firmware_force_wheels_robot_create(
    Charger_t* charger, Chicker_t* chicker, Dribbler_t* dribbler,
    float (*get_robot_position_x)(void), float (*get_robot_position_y)(void),
    float (*get_robot_orientation)(void), float (*get_robot_velocity_x)(void),
    float (*get_robot_velocity_y)(void), float (*get_robot_velocity_angular)(void),
    float (*get_battery_voltage)(void), ForceWheel_t* front_right_wheel,
    ForceWheel_t* front_left_wheel, ForceWheel_t* back_right_wheel,
    ForceWheel_t* back_left_wheel, ControllerState_t* controller_state,
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
 * Get the charger from the given robot
 *
 * @param robot The robot to get the charger from
 *
 * @return The charger from the given robot
 */
Charger_t* app_firmware_robot_getCharger(const FirmwareRobot_t* robot);

/**
 * Get the chicker from the given robot
 *
 * @param robot The robot to get the chicker from
 * @return The chicker from the given robot
 */
Chicker_t* app_firmware_robot_getChicker(const FirmwareRobot_t* robot);

/**
 * Get the dribbler from the given robot
 *
 * @param robot The robot to get the dribbler from
 *
 * @return The dribbler from the given robot
 */
Dribbler_t* app_firmware_robot_getDribbler(const FirmwareRobot_t* robot);

/**
 * Get the x-position of the given robot
 * @param robot The robot to get the y-position for
 *
 * @return The x-position of the given robot, in meters, in global coordinates
 */
float app_firmware_robot_getPositionX(const FirmwareRobot_t* robot);

/**
 * Get the y-position of the given robot
 * @param robot The robot to get the y-position for
 *
 * @return The y-position of the given robot, in meters, in global coordinates
 */
float app_firmware_robot_getPositionY(const FirmwareRobot_t* robot);

/**
 * Get the current orientation of the given robot
 *
 * @param robot The robot to get the orientation of
 *
 * @return The orientation of the robot, in radians
 */
float app_firmware_robot_getOrientation(const FirmwareRobot_t* robot);

/**
 * Get the x-velocity of the given robot
 * @param robot The robot to get the y-velocity for
 *
 * @return The x-velocity of the given robot, in m/s, in global coordinates
 */
float app_firmware_robot_getVelocityX(const FirmwareRobot_t* robot);

/**
 * Get the y-velocity of the given robot
 *
 * @param robot The robot to get the y-velocity for
 *
 * @return The y-velocity of the given robot, in m/s, in global coordinates
 */
float app_firmware_robot_getVelocityY(const FirmwareRobot_t* robot);

/**
 * Get the current angular velocity for the given robot
 *
 * @param robot The robot to get the angular velocity for
 *
 * @return The angular velocity of the given robot, in rad/s
 */
float app_firmware_robot_getVelocityAngular(const FirmwareRobot_t* robot);

/**
 * Get the absolute linear speed of this robot
 * @param robot The robot to get the speed for
 * @return The linear speed of the robot, in m/s
 */
float app_firmware_robot_getSpeedLinear(const FirmwareRobot_t* robot);


/**
 * Get the battery voltage for the given robot
 *
 * @param robot The robot to get the battery voltage for
 *
 * @return The battery voltage for the given robot, in volts
 */
float app_firmware_robot_getBatteryVoltage(const FirmwareRobot_t* robot);

/**
 * Get the constants for this robot
 *
 * @param robot The robot to get constants from
 *
 * @return The constants for the given robot
 */
RobotConstants_t app_firmware_robot_getRobotConstants(const FirmwareRobot_t* robot);

/**
 * Get the controller state for the given robot
 *
 * @param robot The robot to get the controller state for
 *
 * @return A pointer to the controller state for the given robot
 */
ControllerState_t* app_firmware_robot_getControllerState(const FirmwareRobot_t* robot);

// TODO: JavaDoc
void app_firmware_robot_trackVelocityInRobotFrame(FirmwareRobot_t* robot, float linear_velocity_x,
    float linear_velocity_y, float angular_velocity);

void force_wheels_followPosTrajectory(FirmwareRobot_t* robot, PositionTrajectory_t pos_trajectory, size_t trajectory_index);

void velocity_wheels_followPosTrajectory(FirmwareRobot_t* robot, PositionTrajectory_t pos_trajectory, 
    size_t trajectory_index);

void force_wheels_applyDirectPerWheelPower(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg);

void velocity_wheels_applyDirectPerWheelPower(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg);

void force_wheels_setLocalVelocity(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg);

void velocity_wheels_setLocalVelocity(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg);

void force_wheels_stopRobot(FirmwareRobot_t* robot, TbotsProto_StopPrimitive_StopType stop_type);

void velocity_wheels_stopRobot(FirmwareRobot_t* robot, TbotsProto_StopPrimitive_StopType stop_type);

void app_firmware_robot_followPosTrajectory(FirmwareRobot_t* robot, PositionTrajectory_t pos_trajectory, size_t trajectory_index);

void app_firmware_robot_applyDirectPerWheelPower(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectPerWheelControl control_msg);

void app_firmware_robot_setLocalVelocity(FirmwareRobot_t* robot, TbotsProto_DirectControlPrimitive_DirectVelocityControl control_msg);

void app_firmware_robot_stopRobot(FirmwareRobot_t* robot, TbotsProto_StopPrimitive_StopType stop_type);
