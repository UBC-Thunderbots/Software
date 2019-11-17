#pragma once

#include "world/chicker.h"
#include "world/dribbler.h"

// TODO: jdoc for struct and each function
struct Robot;
typedef struct Robot Robot;

Robot* Robot__construct(Chicker* chicker, Dribbler* dribbler);
Chicker* Robot__getChicker(Robot* this);
Dribbler* Robot__getDribbler(Robot* this);
