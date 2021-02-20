#include "software/simulated_tests/non_terminating_validation_functions/robots_slow_down_validation.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsSlowDown(double max_speed, std::shared_ptr<World> world_ptr,
                    ValidationCoroutine::push_type& yield)
{
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        double speed = robot.velocity().length();

        if (speed > max_speed)
        {
            FAIL() << "Robot " + std::to_string(robot.id()) +
                          " is moving faster than 1.5 m/s!";
        }
    }
}
