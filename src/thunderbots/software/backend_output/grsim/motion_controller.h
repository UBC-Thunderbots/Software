//
// Created by evan on 18/08/18.
//

#pragma once

#include "util/constants.h"
#include <utility>

namespace MotionController {

    std::pair<Vector, AngularVelocity>  grSim_bang_bang(Robot robot, Point dest, double desiredFinalSpeed, Angle desiredFinalOrientation, double delaTime);
}


