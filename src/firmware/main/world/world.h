#pragma once

#include "world/robot.h"
#incldue "world/ball.h"

struct World;
typedef struct World World;

World* World_create(Robot* robot);
Robot* World_getRobot(World* this);
void World_destroy(World* world);
