#include "world/firmware_robot.h"

#include <stdlib.h>

struct FirmwareRobot
{
    Chicker* chicker;
    Dribbler* dribbler;
    Wheel* front_right_wheel;
    Wheel* front_left_wheel;
    Wheel* back_right_wheel;
    Wheel* back_left_wheel;
};

FirmwareRobot* FirmwareRobot_create(Chicker* chicker, Dribbler* dribbler,
                                    Wheel* front_right_wheel, Wheel* front_left_wheel,
                                    Wheel* back_right_wheel, Wheel* back_left_wheel)
{
    FirmwareRobot* new_robot = malloc(sizeof(FirmwareRobot));

    new_robot->chicker           = chicker;
    new_robot->dribbler          = dribbler;
    new_robot->front_right_wheel = front_right_wheel;
    new_robot->front_left_wheel  = front_left_wheel;
    new_robot->back_right_wheel  = back_right_wheel;
    new_robot->back_left_wheel   = back_left_wheel;

    return new_robot;
}

void FirmwareRobot_destroy(FirmwareRobot* robot)
{
    free(robot);
}

Chicker* FirmwareRobot_getChicker(FirmwareRobot* robot)
{
    return robot->chicker;
}

Dribbler* FirmwareRobot_getDribbler(FirmwareRobot* robot)
{
    return robot->dribbler;
}

Wheel* FirmwareRobot_getFrontRightWheel(FirmwareRobot* robot)
{
    return robot->front_right_wheel;
}

Wheel* FirmwareRobot_getFrontLeftWheel(FirmwareRobot* robot)
{
    return robot->front_left_wheel;
}

Wheel* FirmwareRobot_getBackRightWheel(FirmwareRobot* robot)
{
    return robot->back_right_wheel;
}

Wheel* FirmwareRobot_getBackLeftWheel(FirmwareRobot* robot)
{
    return robot->back_left_wheel;
}
