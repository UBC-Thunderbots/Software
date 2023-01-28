#include "software/simulated_tests/non_terminating_validation_functions/robots_in_friendly_half_validation.h"

#include "software/logger/logger.h"

void robotsInFriendlyHalf(std::shared_ptr<World> world_ptr,
                          ValidationCoroutine::push_type& yield)
{
    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (!world_ptr->field().pointInFriendlyHalf(robot.position()))
        {
            yield("Robot " + std::to_string(robot.id()) + " entered enemy half");
        }
    }
}

void robotInFriendlyHalf(RobotId robot_id, std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
{
    std::optional<Robot> robot_optional =
        world_ptr->friendlyTeam().getRobotById(robot_id);
    CHECK(robot_optional.has_value())
        << "There is no robot with ID: " + std::to_string(robot_id);
    Point position = robot_optional.value().position();
    if (!world_ptr->field().pointInFriendlyHalf(position))
    {
        yield("Robot " + std::to_string(robot_id) + " entered enemy half");
    }
}
