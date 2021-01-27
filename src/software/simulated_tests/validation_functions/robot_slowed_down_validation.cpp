#include "software/simulated_tests/validation_functions/robot_slowed_down_validation.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotSlowedDown(RobotId robot_id, std::shared_ptr<World> world_ptr,
                      ValidationCoroutine::push_type& yield) {

    auto robot_slowed_down = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robotOptional =
                world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robotOptional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot         = robotOptional.value();
        double speed        = robot.velocity().length();
        const double MAX_SPEED = 1.5;

        return (speed <= MAX_SPEED);
    };

    if (robot_slowed_down(world_ptr))
    {
        yield();
    }
    else {
        LOG(FATAL) << "Robot " + std::to_string(robot_id) + " failed to reach required speed";
    }

}
