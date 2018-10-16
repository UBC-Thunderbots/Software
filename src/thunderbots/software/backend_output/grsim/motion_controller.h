//
// Created by evan on 18/08/18.
//

#pragma once

#include <utility>

#include "util/constants.h"

namespace MotionController
{
    std::pair<Vector, Angle> grSim_bang_bang(Robot robot, Point dest,
                                             double desiredFinalSpeed,
                                             Angle desiredFinalOrientation,
                                             double delaTime);
}
