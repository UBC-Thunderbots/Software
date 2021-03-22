#include "software/simulated_tests/terminating_validation_functions/robot_is_at_orientation_validation.h"

#include "software/logger/logger.h"

void robotIsAtOrientation(RobotId robot_id, std::shared_ptr<World> world_ptr,
                          const Angle& orientation,
                          const Angle& close_to_orientation_threshold,
                          ValidationCoroutine::push_type& yield)
{
    auto robot_is_at_orientation = [robot_id, orientation,
                                    close_to_orientation_threshold](
                                       std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot = robot_optional.value();
        return robot.orientation().minDiff(orientation) < close_to_orientation_threshold;
    };

    while (!robot_is_at_orientation(world_ptr))
    {
        std::stringstream ss;
        ss << orientation;
        yield("Robot " + std::to_string(robot_id) + " does not have orientation of " +
              ss.str());
    }
}
