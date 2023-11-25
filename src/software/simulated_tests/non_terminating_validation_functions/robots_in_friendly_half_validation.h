#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the friendly robots are in the friendly half of the field, else assertion
 * fails.
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotsInFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield);

/**
 * Checks if a friendly robot is in the friendly half of the field, else assertion fails.
 * @param robot_id the ID of the robot in question, there must exist a robot
 * for the given robot_id
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotInFriendlyHalf(RobotId robot_id, std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield);
