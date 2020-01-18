#include "app/world/firmware_robot.h"

#include <stdlib.h>

struct FirmwareRobot
{
    Chicker_t* chicker;
    Dribbler_t* dribbler;
    float (*get_robot_position_x)();
    float (*get_robot_position_y)();
    Wheel_t* front_right_wheel;
    Wheel_t* front_left_wheel;
    Wheel_t* back_right_wheel;
    Wheel_t* back_left_wheel;
};

FirmwareRobot_t* app_firmware_robot_create(Chicker_t* chicker, Dribbler_t* dribbler,

                                           float (*get_robot_position_x)(),
                                           float (*get_robot_position_y)(),
                                           Wheel_t* front_right_wheel,
                                           Wheel_t* front_left_wheel,
                                           Wheel_t* back_right_wheel,
                                           Wheel_t* back_left_wheel)
{
    FirmwareRobot_t* new_robot = malloc(sizeof(FirmwareRobot_t));

    new_robot->chicker              = chicker;
    new_robot->dribbler             = dribbler;
    new_robot->get_robot_position_x = get_robot_position_x;
    new_robot->get_robot_position_y = get_robot_position_y;
    new_robot->front_right_wheel    = front_right_wheel;
    new_robot->front_left_wheel     = front_left_wheel;
    new_robot->back_right_wheel     = back_right_wheel;
    new_robot->back_left_wheel      = back_left_wheel;

    return new_robot;
}

void app_firmware_robot_destroy(FirmwareRobot_t* robot)
{
    free(robot);
}

Chicker_t* app_firmware_robot_getChicker(FirmwareRobot_t* robot)
{
    return robot->chicker;
}

Dribbler_t* app_firmware_robot_getDribbler(FirmwareRobot_t* robot)
{
    return robot->dribbler;
}

float app_firmware_robot_getPositionX(FirmwareRobot_t* robot)
{
    return robot->get_robot_position_x();
}

float app_firmware_robot_getPositionY(FirmwareRobot_t* robot)
{
    return robot->get_robot_position_y();
}

Wheel_t* app_firmware_robot_getFrontRightWheel(FirmwareRobot_t* robot)
{
    return robot->front_right_wheel;
}

Wheel_t* app_firmware_robot_getFrontLeftWheel(FirmwareRobot_t* robot)
{
    return robot->front_left_wheel;
}

Wheel_t* app_firmware_robot_getBackRightWheel(FirmwareRobot_t* robot)
{
    return robot->back_right_wheel;
}

Wheel_t* app_firmware_robot_getBackLeftWheel(FirmwareRobot_t* robot)
{
    return robot->back_left_wheel;
}
