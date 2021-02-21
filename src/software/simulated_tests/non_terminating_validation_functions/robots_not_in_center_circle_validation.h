#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the friendly robots are not in center circle of field, else fails assertion.
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotsNotInCenterCircle(std::shared_ptr<World> world_ptr,
                             ValidationCoroutine::push_type& yield);

/**
 * Checks if a friendly robot is not in center circle of field, else fails assertion.
 * @param robot the robot in question
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotNotInCenterCircle(Robot robot, std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield);
