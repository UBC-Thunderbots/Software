#include "software/simulated_tests/non_terminating_validation_functions/robots_avoid_ball_validation.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsAvoidBall(std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        double distance = (robot.position() - world_ptr->ball().position()).length();
        const double MIN_DISTANCE_FROM_BALL = ROBOT_MAX_RADIUS_METERS + 0.5;

        if (distance < MIN_DISTANCE_FROM_BALL)
        {
            FAIL() << "Robot " + std::to_string(robot.id()) +
                          " less than 0.5 m away from the ball!";
        }
    }
}
