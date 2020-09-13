#include "software/simulated_tests/validation_functions/robot_received_ball_validation.h"
#include "software/ai/evaluation/robot.h"

void robotReceivedBall(Robot robot, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield)
{
    auto robot_received_ball = [robot](std::shared_ptr<World> world_ptr) {
        std::optional<bool> result = robotHasPossession(world_ptr->ball().getPreviousStates(), robot.getPreviousStates());
        bool robot_has_possession = result.value();

        return robot_has_possession;
    };

    while (!robot_received_ball(world_ptr))
    {
        yield();
    }
}
