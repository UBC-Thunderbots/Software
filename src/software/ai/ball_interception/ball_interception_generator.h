#pragma once

#include "software/world/world.h"

Point generateBallInterceptionPoint(const Field& field, const Ball& ball,
                                    const Robot& robot);

Point interceptFastBall(const Field& field, const Ball& ball, const Robot& robot);

Point interceptSlowBall(const Field& field, const Ball& ball, const Robot& robot);
