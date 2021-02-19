#include "software/simulated_tests/validation_functions/robots_slow_down_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsSlowDown(std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        int robot_id           = robot.id();
        double speed           = robot.velocity().length();
        const double MAX_SPEED = 1.5;

        bool robot_slowed_down = speed <= MAX_SPEED;
        if (!robot_slowed_down)
        {
            // TODO: change from throwing exception to FAIL(), once PR #1946 is merged
            throw std::runtime_error("Robot " + std::to_string(robot_id) +
                                     " did not slow down!");
        }
    }
}
