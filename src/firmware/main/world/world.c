#include "world/world.h"

#include <stdlib.h>

struct World
{
    Robot* robot;
};

World* World_create(Robot* robot)
{
    World* new_world = malloc(sizeof(World));

    new_world->robot = robot;

    return new_world;
}

Robot* World_getRobot(World* this)
{
    return this->robot;
}

void World_destroy(World* world){
    free(world);
}
