#include "world/robot.h"

#include <stdlib.h>

struct Robot
{
    Chicker* chicker;
    Dribbler* dribbler;
};

Robot* Robot_create(Chicker* chicker, Dribbler* dribbler)
{
    Robot* new_robot = malloc(sizeof(Robot));

    new_robot->chicker  = chicker;
    new_robot->dribbler = dribbler;

    return new_robot;
}

Chicker* Robot_getChicker(Robot* robot)
{
    return robot->chicker;
}

Dribbler* Robot_getDribbler(Robot* robot)
{
    return robot->dribbler;
}

void Robot_destroy(Robot* robot){
    free(robot);
}
