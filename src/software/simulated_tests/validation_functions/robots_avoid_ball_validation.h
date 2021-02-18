#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if robots are keeping appropriate distance from
 * the ball when the Referee has issued a STOP command
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotsAvoidBall(std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield);
