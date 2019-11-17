#pragma once

#include "world/robot.h"

struct World;
typedef struct World World;

World* World__construct(Robot* robot);
Robot* World__getRobot(World* this);
