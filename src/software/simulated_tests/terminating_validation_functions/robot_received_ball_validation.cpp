#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"

#include "software/logger/logger.h"


void robotReceivedBall(std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield)
{
<<<<<<< HEAD
    auto ball_near_dribbler = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);

        Robot robot         = robot_optional.value();
        Point ball_position = world_ptr->ball().position();
        return robot.isNearDribbler(ball_position, 0.05);
=======
    auto ball_near_dribbler = [](std::shared_ptr<World> world_ptr) {
        std::vector<Robot> robots = world_ptr->friendlyTeam().getAllRobots();
        return std::any_of(robots.begin(),robots.end(),[world_ptr](Robot robot){
            Point ball_position = world_ptr->ball().position();
            return robot.isNearDribbler(ball_position, 0.01);
        });
>>>>>>> Modified 2 validation functions and simulated tests
    };

    while (!ball_near_dribbler(world_ptr))
    {
        yield("No robot has received the ball");
    }
}
