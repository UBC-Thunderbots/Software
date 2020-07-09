#include "software/simulated_tests/validation_functions/one_meter_from_ball_validation.h"

void oneMeterFromBall(std::shared_ptr<World> world_ptr,
                      ValidationCoroutine::push_type& yield)
{
    auto friendly_robots_1_meter_from_ball = [](std::shared_ptr<World> world_ptr) {
        Point ball_position = world_ptr->ball().position();
        for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
        {
            double abs_error =
                std::fabs((robot.position() - ball_position).length() - 1.0);
            if (abs_error > 0.05)
            {
                return false;
            }
        }
        return true;
    };

    while (!friendly_robots_1_meter_from_ball(world_ptr))
    {
        yield();
    }
}