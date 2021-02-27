#include "software/simulated_tests/terminating_validation_functions/robots_away_from_ball_validation.h"

void robotsAwayFromBall(double min_distance, std::vector<RobotId> excluded_robots,
                        std::shared_ptr<World> world_ptr,
                        ValidationCoroutine::push_type& yield)
{
    auto all_robots_except_excluded_away_from_ball =
        [excluded_robots](std::shared_ptr<World> world_ptr) {
            Point ball_position = world_ptr->ball().position();
            for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
            {
                if ((std::find(excluded_robots.begin(), excluded_robots.end(),
                               robot.id()) == excluded_robots.end()) &&
                    ((robot.position() - ball_position).length() < 1))
                {
                    return false;
                }
            }
            return true;
        };

    while (!all_robots_except_excluded_away_from_ball(world_ptr))
    {
        yield();
    }
}
