#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"

#include "software/logger/logger.h"


void robotReceivedBall(RobotId robot_id, std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield)
{
    auto ball_near_dribbler = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot         = robot_optional.value();
        Point ball_position = world_ptr->ball().position();
        return robot.isNearDribbler(ball_position, 0.05);
    };

    while (!ball_near_dribbler(world_ptr))
    {
        yield("Robot with ID " + std::to_string(robot_id) + " has not received the ball");
    }
}
