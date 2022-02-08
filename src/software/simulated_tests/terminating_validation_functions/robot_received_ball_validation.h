#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if any robot has received the ball at any point in the test
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotReceivedBall(std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield);
