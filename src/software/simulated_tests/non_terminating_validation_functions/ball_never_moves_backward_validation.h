#pragma once

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks that the ball doesn't move backward.
 *
 * @param world_ptr     the world pointer given by the simulator. Gets updated every tick
 * @param yield         yields control to the next routine (coroutines)
 * @param tolerance     tolerance given to robot's x-coordinate changes
 */
void ballNeverMovesBackward(std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield,
                            double tolerance = 0.04);
