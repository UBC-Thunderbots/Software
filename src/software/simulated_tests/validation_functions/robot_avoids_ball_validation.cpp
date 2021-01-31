#include "software/simulated_tests/validation_functions/robot_avoids_ball_validation.h"
#include "software/geom/algorithms/contains.h"
#include "software/logger/logger.h"

void robotAvoidsBall(RobotId robot_id, std::shared_ptr<World> world_ptr,
                     ValidationCoroutine::push_type& yield) {

    auto robot_avoids_ball = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robotOptional =
                world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robotOptional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot                         = robotOptional.value();
        double distance_from_ball           = (robot.position() - world_ptr->ball().position()).length();
        const double MIN_DISTANCE_FROM_BALL = ROBOT_MAX_RADIUS_METERS + 0.5;

        return (distance_from_ball <= MIN_DISTANCE_FROM_BALL);
    };

    if (!robot_avoids_ball(world_ptr))
    {
        LOG(FATAL) << "Robot " + std::to_string(robot_id) + " failed to avoid the ball";
    }
    else {
        LOG(WARNING) << "Robot " + std::to_string(robot_id) + " is avoiding the ball!";
    }

}
