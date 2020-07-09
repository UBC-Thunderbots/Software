#include "software/simulated_tests/validation_functions/distance_to_ball_kick_off_validation.h"

void robotTooCloseDuringEnemyKickOff(std::shared_ptr<World> world_ptr,
                                     ValidationCoroutine::push_type& yield)
{
    auto robot_too_close = [](std::shared_ptr<World> world_ptr) {
        GameState game_state = world_ptr->gameState();
        if (game_state.isTheirKickoff())
        {
            Point ball_position = world_ptr->ball().position();
            for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
            {
                double abs_error =
                    std::fabs((robot.position() - ball_position).length() - 1.0);
                // Friendly robots must not be < 0.5 meters to the ball during enemy
                // kick-off
                if (abs_error > 0.025)
                {
                    return false;
                }
            }
            return true;
        }

        return true;
    };

    while (!robot_too_close(world_ptr))
    {
        yield();
    }
}