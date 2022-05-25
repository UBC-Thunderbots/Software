#include "firmware/app/world/firmware_world.h"

#include <stdlib.h>

struct FirmwareWorld
{
    FirmwareRobot_t* robot;
    FirmwareBall_t* ball;
    float (*get_current_time_seconds)(void);
};

FirmwareWorld_t* app_firmware_world_create(FirmwareRobot_t* robot, FirmwareBall_t* ball,
                                           float (*get_current_time_seconds)(void))
{
    FirmwareWorld_t* new_world = malloc(sizeof(FirmwareWorld_t));

    new_world->robot                    = robot;
    new_world->ball                     = ball;
    new_world->get_current_time_seconds = get_current_time_seconds;

    return new_world;
}

void app_firmware_world_destroy(FirmwareWorld_t* world)
{
    free(world);
}

FirmwareRobot_t* app_firmware_world_getRobot(FirmwareWorld_t* world)
{
    return world->robot;
}

FirmwareBall_t* app_firmware_world_getBall(FirmwareWorld_t* world)
{
    return world->ball;
}

float app_firmware_world_getCurrentTime(FirmwareWorld_t* world)
{
    return world->get_current_time_seconds();
}
