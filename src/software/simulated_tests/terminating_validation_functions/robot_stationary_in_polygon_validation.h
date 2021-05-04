#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the robot represented by robot_id is stationary in the polygon for num_ticks
 * Used to check if robot is not oscillating after reaching destination
 * @param robot_id the ID of the robot in question, there must exist a robot
 * for the given robot_id
 * @param polygon the polygon the robot should be within
 * @param num_ticks how many consecutive ticks the robot must be stationary for
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotStationaryInPolygon(RobotId robot_id, Polygon polygon, unsigned int num_ticks,
                              std::shared_ptr<World> world_ptr,
                              ValidationCoroutine::push_type& yield);
