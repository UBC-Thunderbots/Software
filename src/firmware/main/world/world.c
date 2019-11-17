#include "world/world.h"

#include <stdlib.h>

struct World
{
    Robot* robot;
};

World* World__construct(Robot* robot){
    World* new_world = malloc(sizeof(World));

    new_world->robot = robot;

    return new_world;
}

Robot* World__getRobot(World* this){
    return this->robot;
}
