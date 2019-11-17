#include "world/robot.h"

#include <stdlib.h>

struct Robot
{
    Chicker* chicker;
    Dribbler* dribbler;
};

Robot* Robot__construct(Chicker* chicker, Dribbler* dribbler){
    Robot* new_robot = malloc(sizeof(Robot));

    new_robot->chicker = chicker;
    new_robot->dribbler = dribbler;

    return new_robot;
}

Chicker* Robot__getChicker(Robot* this){
    return this->chicker;
}

Dribbler* Robot__getDribbler(Robot* this){
    return this->dribbler;
}

