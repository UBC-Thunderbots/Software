#include "software/simulated_tests/non_terminating_validation_functions/robots_avoid_ball_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsAvoidBall(double min_distance, std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        double current_distance =
            (robot.position() - world_ptr->ball().position()).length() -
            ROBOT_MAX_RADIUS_METERS;

        if (current_distance < min_distance)
        {
            yield("Robot " + std::to_string(robot.id()) + " is less than " +
                  std::to_string(min_distance) + " m away from the ball!");
        }
    }
}
