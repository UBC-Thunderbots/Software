#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if robots have slowed down to the required speed
 * when the Referee has issued a STOP command
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotSlowedDown(RobotId robot_id, std::shared_ptr<World> world_ptr,
                        ValidationCoroutine::push_type& yield);