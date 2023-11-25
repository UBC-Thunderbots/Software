#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the ball is at the point
 * @param point the point that the ball is expected to be at
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void ballAtPoint(Point point, std::shared_ptr<World> world_ptr,
                 ValidationCoroutine::push_type& yield);
