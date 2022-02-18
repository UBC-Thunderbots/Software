#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if a friendly robot is in center circle of field, else fails assertion.
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotInCenterCircle(std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield);
