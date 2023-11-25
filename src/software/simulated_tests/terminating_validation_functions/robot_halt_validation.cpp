#include "software/simulated_tests/terminating_validation_functions/robot_halt_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotHalt(std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        double robot_speed = robot.velocity().length();

        if (robot_speed > 1e-3)
        {
            yield("Robot " + std::to_string(robot.id()) +
                  " has not stopped and is moving at " + std::to_string(robot_speed));
        }
    }
}
