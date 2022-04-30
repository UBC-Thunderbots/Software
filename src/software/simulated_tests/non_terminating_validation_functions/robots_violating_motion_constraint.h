#pragma once
#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the friendly robots do not enter specified motion constraint during the test.
 * If they do, assertion fails.
 *
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines) with error message
 * @param obstacle_factory the obstacle factory object to be passed in
 * @param constraint the motion constraint to be specified
 */
void robotsViolatingMotionConstraint(
    std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield,
    std::shared_ptr<RobotNavigationObstacleFactory> obstacle_factory,
    TbotsProto::MotionConstraint constraint);
