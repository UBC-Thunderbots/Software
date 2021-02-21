#include "software/simulated_tests/terminating_validation_functions/robot_in_center_circle_validation.h"

#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotInCenterCircle(RobotId robot_id, std::shared_ptr<World> world_ptr,
                         ValidationCoroutine::push_type& yield)
{
    auto robot_in_center_circle = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robotOptional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robotOptional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Point position = robotOptional.value().position();
        return contains(world_ptr->field().centerCircle(), position);
    };

    while (!robot_in_center_circle(world_ptr))
    {
        yield();
    }
}
