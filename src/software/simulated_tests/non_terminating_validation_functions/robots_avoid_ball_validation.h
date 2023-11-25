#pragma once

#include <algorithm>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if robots are keeping at least the minimum required distance from
 * the ball
 * @param min_distance the minimum required distance that robots must be from the ball
 * @param excluded_robots RobotIds to ignore in this validation function's checking
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotsAvoidBall(double min_distance, std::vector<RobotId> excluded_robots,
                     std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield);
