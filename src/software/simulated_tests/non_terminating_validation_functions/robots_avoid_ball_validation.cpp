#include "software/simulated_tests/non_terminating_validation_functions/robots_avoid_ball_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsAvoidBall(double min_distance, std::vector<RobotId> excluded_robots,
                     std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield)
{
    for (size_t i = 0; i < world_ptr->friendlyTeam().getAllRobots().size(); i++)
    {
        RobotId id = world_ptr->friendlyTeam().getAllRobots().at(i).id();
        double current_distance =
            (world_ptr->friendlyTeam().getAllRobots().at(i).position() -
             world_ptr->ball().position())
                .length() -
            ROBOT_MAX_RADIUS_METERS;

        if (current_distance < min_distance &&
            std::find(excluded_robots.begin(), excluded_robots.end(), id) ==
                excluded_robots.end())
        {
            yield("Robot " + std::to_string(id) + " is less than " +
                  std::to_string(min_distance) + " m away from the ball!");
        }
    }
}
