#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the friendly robots are in friendly half of the field, else assertion fails.
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotsInFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield);
