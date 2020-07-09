#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

void friendlyTouchedBallInEnemyDefense(std::shared_ptr<World> world_ptr,
                                       ValidationCoroutine::push_type& yield);
