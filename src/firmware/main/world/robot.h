#pragma once

#include "world/chicker.h"
#include "world/dribbler.h"

// TODO: jdoc for struct and each function
struct Robot;
typedef struct Robot Robot;

Robot* Robot_create(Chicker* chicker, Dribbler* dribbler);
Chicker* Robot_getChicker(Robot* robot);
Dribbler* Robot_getDribbler(Robot* robot);
void Robot_destroy(Robot* robot);
