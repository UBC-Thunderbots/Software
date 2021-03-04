#include "software/logger/logger.h"
#include "software/simulated_tests/terminating_validation_functions/robot_received_ball_validation.h"

void robotAtPosition(RobotId robot_id, std::shared_ptr<World> world_ptr,
                     const Point& destination, double close_to_destination_threshold,
                     ValidationCoroutine::push_type& yield)
{
    auto robot_at_destination = [robot_id, destination, close_to_destination_threshold](
                                    std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robotOptional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robotOptional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot = robotOptional.value();
        return (robot.position() - destination).length() < close_to_destination_threshold;
    };

    while (!robot_at_destination(world_ptr))
    {
        std::stringstream ss;
        ss << destination;
        yield("Robot with ID " + std::to_string(robot_id) + " is not at position " +
              ss.str());
    }
}
