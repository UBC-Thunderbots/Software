#include "software/simulated_tests/non_terminating_validation_functions/robot_not_excessively_dribbling_validation.h"

#include "software/logger/logger.h"

void robotNotExcessivelyDribbling(RobotId robot_id, std::shared_ptr<World> world_ptr,
                                  ValidationCoroutine::push_type& yield)
{
    auto robot_has_ball_in_dribbler = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);

        return robot_optional.value().isNearDribbler(world_ptr->ball().position());
    };

    Point continous_dribbling_start_point  = world_ptr->ball().position();
    bool ball_in_dribbler_in_previous_tick = false;

    while (!robot_has_ball_in_dribbler(world_ptr) ||
           (world_ptr->ball().position() - continous_dribbling_start_point).length() <=
               1.0)
    {
        yield("");
        if (!robot_has_ball_in_dribbler(world_ptr) ||
            (robot_has_ball_in_dribbler(world_ptr) && !ball_in_dribbler_in_previous_tick))
        {
            continous_dribbling_start_point = world_ptr->ball().position();
        }
        ball_in_dribbler_in_previous_tick = robot_has_ball_in_dribbler(world_ptr);
    }
    yield("Ball was dribbled for more than 1 meter");
}
