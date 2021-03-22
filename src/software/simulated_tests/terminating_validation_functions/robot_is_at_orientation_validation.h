#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the robot represented by robot_id has arrived at the given orientation at
 * any point in the test
 * @param robot_id the ID of the robot in question, there must exist a robot
 * for the given robot_id
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param orientation The expected orientation that the robot is at
 * @param close_to_orientation_threshold The threshold where the robot is considered close
 * enough to the expected orientation
 * @param yield yields control to the next routine (coroutines)
 */
void robotIsAtOrientation(RobotId robot_id, std::shared_ptr<World> world_ptr,
                          const Angle& orientation,
                          const Angle& close_to_orientation_threshold,
                          ValidationCoroutine::push_type& yield);
