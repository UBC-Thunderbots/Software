#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the robot represented by robot_id is in the polygon
 * @param robot_id the ID of the robot in question, there must exist a robot
 * for the given robot_id
 * @param polygon the polygon the robot should be within
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 *
 * @return true if the polygon contains the robot
 */
bool robotInPolygon(RobotId robot_id, Polygon polygon, std::shared_ptr<World> world_ptr);
