#pragma once

extern "C"
{
#include "firmware/app/primitives/primitive_manager.h"
#include "firmware/app/world/chicker.h"
#include "firmware/app/world/dribbler.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
#include "firmware/app/world/force_wheel.h"
#include "firmware/shared/physics.h"
#include "shared/proto/primitive.nanopb.h"
}

/**
 * Because the FirmwareRobot_t struct is defined in the .c file (rather than the .h file),
 * C++ considers it an incomplete type and is unable to use it with smart pointers
 * because it doesn't know the size of the object. Therefore we need to create our own
 * "Deleter" class we can provide to the smart pointers to handle that instead.
 *
 * See https://en.cppreference.com/w/cpp/memory/unique_ptr/unique_ptr for more info and
 * examples
 */
struct FirmwareRobotDeleter
{
    void operator()(FirmwareRobot_t* firmware_robot) const
    {
        ForceWheel_t* front_left_wheel = app_firmware_robot_getFrontLeftWheel(firmware_robot);
        app_force_wheel_destroy(front_left_wheel);

        ForceWheel_t* back_left_wheel = app_firmware_robot_getBackLeftWheel(firmware_robot);
        app_force_wheel_destroy(back_left_wheel);

        ForceWheel_t* back_right_wheel = app_firmware_robot_getBackRightWheel(firmware_robot);
        app_force_wheel_destroy(back_right_wheel);

        ForceWheel_t* front_right_wheel =
            app_firmware_robot_getFrontRightWheel(firmware_robot);
        app_force_wheel_destroy(front_right_wheel);

        Chicker_t* chicker = app_firmware_robot_getChicker(firmware_robot);
        app_chicker_destroy(chicker);

        Charger_t* charger = app_firmware_robot_getCharger(firmware_robot);
        app_charger_destroy(charger);

        Dribbler_t* dribbler = app_firmware_robot_getDribbler(firmware_robot);
        app_dribbler_destroy(dribbler);

        ControllerState_t* controller_state =
            app_firmware_robot_getControllerState(firmware_robot);
        delete controller_state;

        app_firmware_robot_destroy(firmware_robot);
    };
};

struct FirmwareWorldDeleter
{
    void operator()(FirmwareWorld_t* firmware_world) const
    {
        FirmwareRobot_t* firmware_robot = app_firmware_world_getRobot(firmware_world);

        FirmwareRobotDeleter()(firmware_robot);

        FirmwareBall_t* firmware_ball = app_firmware_world_getBall(firmware_world);
        app_firmware_ball_destroy(firmware_ball);

        app_firmware_world_destroy(firmware_world);
    };
};

struct FirmwarePrimitiveManagerDeleter
{
    void operator()(PrimitiveManager_t* primitive_manager) const
    {
        app_primitive_manager_destroy(primitive_manager);
    };
};

struct FirmwareBallDeleter
{
    void operator()(FirmwareBall_t* firmware_ball) const
    {
        app_firmware_ball_destroy(firmware_ball);
    };
};
