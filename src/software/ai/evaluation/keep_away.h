#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/pass.h"
#include "software/world/world.h"

Point findKeepAwayTargetPoint(const Point& ball_possessor_point,
                              const Pass& best_pass_so_far, const World& world);
