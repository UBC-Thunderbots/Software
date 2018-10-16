//
// Created by evan on 18/08/18.
//

#pragma once

#include <utility>

#include "util/constants.h"

namespace MotionController
{
    std::pair<Vector, Angle> grSimBangBang(const Robot robot, const Point dest,
                                           const double desired_final_speed,
                                           const Angle desired_final_orientation,
                                           double delta_time);
}
