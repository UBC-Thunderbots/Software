#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"

#include "software/logger/logger.h"


void robotReceivedBall(std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield)
{
    auto ball_near_dribbler = [](std::shared_ptr<World> world_ptr) {
        std::vector<Robot> robots = world_ptr->friendlyTeam().getAllRobots();
        return std::any_of(robots.begin(), robots.end(), [world_ptr](Robot robot) {
            Point ball_position = world_ptr->ball().position();
            return robot.isNearDribbler(ball_position, 0.02);
        });
    };

    while (!ball_near_dribbler(world_ptr))
    {
        yield("No robot has received the ball");
    }
}
