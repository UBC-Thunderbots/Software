#include "world/firmware_world.h"

#include <stdlib.h>

struct FirmwareWorld
{
    FirmwareRobot* robot;
};

FirmwareWorld* FirmwareWorld_create(FirmwareRobot* robot)
{
    FirmwareWorld* new_world = malloc(sizeof(FirmwareWorld));

    new_world->robot = robot;

    return new_world;
}


void FirmwareWorld_destroy(FirmwareWorld* world)
{
    free(world);
}

FirmwareRobot* FirmwareWorld_getRobot(FirmwareWorld* world)
{
    return world->robot;
}

