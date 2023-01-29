#include "software/simulated_tests/non_terminating_validation_functions/robots_not_in_center_circle_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotsNotInCenterCircle(std::shared_ptr<World> world_ptr,
                             ValidationCoroutine::push_type& yield)
{
    for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
    {
        if (contains(world_ptr->field().centerCircle(), robot.position()))
        {
            yield("Robot " + std::to_string(robot.id()) + " entered center circle");
        }
    }
}

void robotNotInCenterCircle(RobotId robot_id, std::shared_ptr<World> world_ptr,
                            ValidationCoroutine::push_type& yield)
{
    std::optional<Robot> robot_optional =
        world_ptr->friendlyTeam().getRobotById(robot_id);
    CHECK(robot_optional.has_value())
        << "There is no robot with ID: " + std::to_string(robot_id);
    Point position = robot_optional.value().position();
    if (contains(world_ptr->field().centerCircle(), position))
    {
        yield("Robot " + std::to_string(robot_id) + " entered center circle");
    }
}
