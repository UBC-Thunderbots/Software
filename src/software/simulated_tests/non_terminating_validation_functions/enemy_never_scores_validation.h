#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks that the ball never enters the friendly goal
 *
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void enemyNeverScores(std::shared_ptr<World> world_ptr,
                      ValidationCoroutine::push_type& yield);
