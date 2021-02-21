#pragma once

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the ball has been scored or is in play.
 *
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 **/
void ballInPlay(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield);
