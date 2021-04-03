#pragma once

#include <algorithm>

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks the the goalie never leaves the friendly defense area
 *
 * @param goalie_id the robot id of the goalie
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void goalieNeverLeavesDefenseArea(RobotId goalie_id, std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield);
