#include "app/world/firmware_world.h"

#include <stdlib.h>

struct FirmwareWorld
{
    FirmwareRobot* robot;
    FirmwareBall* ball;
};

FirmwareWorld* app_firmware_world_create(FirmwareRobot* robot, FirmwareBall* ball)
{
    FirmwareWorld* new_world = malloc(sizeof(FirmwareWorld));

    new_world->robot = robot;
    new_world->ball  = ball;

    return new_world;
}

void app_firmware_world_destroy(FirmwareWorld* world)
{
    free(world);
}

FirmwareRobot* app_firmware_world_getRobot(FirmwareWorld* world)
{
    return world->robot;
}

FirmwareBall* app_firmware_world_getBall(FirmwareWorld* world)
{
    return world->ball;
}
