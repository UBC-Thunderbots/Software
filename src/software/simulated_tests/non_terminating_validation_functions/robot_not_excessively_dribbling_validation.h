#pragma once

#include <gtest/gtest.h>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks that the robot does not dribble for more than 1 meter at a time
 *
 * @param robot_id the robot_id to check
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotNotExcessivelyDribbling(RobotId robot_id, std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield);
