#pragma once
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/motion_constraint/motion_constraint.h"

/**
 * Checks if the friendly robots are in the friendly half of the field, else assertion
 * fails.
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 */
void robotsViolatingMotionConstraint(std::shared_ptr<World> world_ptr,
                                        ValidationCoroutine::push_type& yield,
                                        std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory,
                                        MotionConstraint constraint);