#include "world/firmware_world.h"

#include <stdlib.h>

struct FirmwareWorld
{
    FirmwareRobot* robot;
    FirmwareBall* ball;
};

FirmwareWorld* FirmwareWorld_create(FirmwareRobot* robot, FirmwareBall* ball)
{
    FirmwareWorld* new_world = malloc(sizeof(FirmwareWorld));

    new_world->robot = robot;
    new_world->ball = robot;

    return new_world;
}


void FirmwareWorld_destroy(FirmwareWorld* world){
    free(world);
}

FirmwareRobot* FirmwareWorld_getFirmwareRobot(FirmwareWorld* world)
{
    return world->robot;
}

FirmwareBall* FirmwareWorld_getFirmwareBall(FirmwareWorld* world)
{
    return world->ball;
}
