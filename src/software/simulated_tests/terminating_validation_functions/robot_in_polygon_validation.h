#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the some given number of robots are in the polygon
 * @param polygon the polygon the robot should be within
 * @param count the number of robots in the polygon
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotInPolygon(Polygon polygon, int count, std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield);
