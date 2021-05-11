#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if robots have slowed down to the maximum allowed speed
 * @param max_speed the maximum speed that robots are allowed to move
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotsSlowDown(double max_speed, std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield);
