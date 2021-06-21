#include "software/simulated_tests/terminating_validation_functions/robot_halt_with_delay_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotHaltWithDelay(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    while (world_ptr->getMostRecentTimestamp() < Timestamp::fromSeconds(9.5))
    {
        // stop test from passing immediately
    }
    for (auto robot : world_ptr->friendlyTeam().getAllRobots())
    {
        double robot_speed = robot.velocity().length();

        if (robot_speed > 0)
        {
            yield("Robot " + std::to_string(robot.id()) + " has not stopped");
        }
    }
}