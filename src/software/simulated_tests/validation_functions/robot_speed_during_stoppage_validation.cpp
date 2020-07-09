#include "software/simulated_tests/validation_functions/robot_speed_during_stoppage_validation.h"

void robotTooFastDuringStoppage(std::shared_ptr<World> world_ptr,
                                ValidationCoroutine::push_type& yield)
{
    auto robot_too_fast = [](std::shared_ptr<World> world_ptr) {
        GameState game_state = world_ptr->gameState();
        if (game_state.isStopped())
        {
            for (const auto& robot : world_ptr->friendlyTeam().getAllRobots())
            {
                Vector robot_velocity = robot.velocity();
                // Robots cannot exceed a speed of 1.5 m/s during stoppage as regulated by
                // the SSL
                // TODO grace period of 2 seconds to slow down
                if (robot_velocity.length() >= 1.5)
                {
                    return false;
                }
            }
            return true;
        }

        return true;
    };

    while (!robot_too_fast(world_ptr))
    {
        yield();
    }
}