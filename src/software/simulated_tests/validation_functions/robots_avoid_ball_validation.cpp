#include "software/simulated_tests/validation_functions/robots_avoid_ball_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsAvoidBall(std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        int robot_id = robot.id();
        double distance_from_ball =
            (robot.position() - world_ptr->ball().position()).length();
        const double MIN_DISTANCE_FROM_BALL = ROBOT_MAX_RADIUS_METERS + 0.5;

        bool robot_avoided_ball = distance_from_ball >= MIN_DISTANCE_FROM_BALL;
        if (!robot_avoided_ball)
        {
            // TODO: change from throwing exception to FAIL(), once PR #1946 is merged
            throw std::runtime_error("Robot " + std::to_string(robot_id) +
                                     " did not avoid the ball!");
        }
    }
}
