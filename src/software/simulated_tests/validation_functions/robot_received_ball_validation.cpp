#include "software/simulated_tests/validation_functions/robot_received_ball_validation.h"
#include "software/ai/evaluation/robot.h"

#include "software/logger/logger.h"


void robotReceivedBall(RobotId robot_id, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    auto ball_near_dribbler = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot = world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot.has_value())
        {
            LOG(FATAL);
        }

        Point ball_position = world_ptr->ball().position();
        return pointNearRobot(ball_position, robot.value());
    };

    while (!ball_near_dribbler(world_ptr))
    {
        yield();
    }
}
