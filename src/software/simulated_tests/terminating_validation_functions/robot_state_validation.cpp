#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"

#include "software/logger/logger.h"

void robotAtPosition(RobotId robot_id, std::shared_ptr<World> world_ptr,
                     const Point& destination, double close_to_destination_threshold,
                     ValidationCoroutine::push_type& yield)
{
    auto robot_at_destination = [robot_id, destination, close_to_destination_threshold](
                                    std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot = robot_optional.value();
        return (robot.position() - destination).length() < close_to_destination_threshold;
    };

    while (!robot_at_destination(world_ptr))
    {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        std::stringstream ss;
        ss << "Robot with ID " << robot_id << " is at position "
           << robot_optional->position() << ", expected to be at " << destination;
        yield(ss.str());
    }
}

void robotAtOrientation(RobotId robot_id, std::shared_ptr<World> world_ptr,
                        const Angle& orientation,
                        const Angle& close_to_orientation_threshold,
                        ValidationCoroutine::push_type& yield)
{
    std::optional<Robot> robot_optional =
        world_ptr->friendlyTeam().getRobotById(robot_id);
    if (!robot_optional.has_value())
    {
        LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
    }

    while ((robot_optional->orientation().minDiff(orientation) >
            close_to_orientation_threshold))
    {
        robot_optional = world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }
        Robot robot = robot_optional.value();
        std::stringstream ss;
        ss << "Robot " << std::to_string(robot_id) << " does not have orientation of "
           << orientation << ", instead it has orientation of " << robot.orientation();
        yield(ss.str());
    }
}

void robotAtAngularVelocity(RobotId robot_id, std::shared_ptr<World> world_ptr,
                            const AngularVelocity& angular_velocity,
                            const AngularVelocity& close_to_angular_velocity_threshold,
                            ValidationCoroutine::push_type& yield)
{
    auto robot_is_at_angular_velocity =
        [robot_id, angular_velocity,
         close_to_angular_velocity_threshold](std::shared_ptr<World> world_ptr) {
            std::optional<Robot> robot_optional =
                world_ptr->friendlyTeam().getRobotById(robot_id);
            if (!robot_optional.has_value())
            {
                LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
            }

            Robot robot = robot_optional.value();
            return robot.angularVelocity().minDiff(angular_velocity) <
                   close_to_angular_velocity_threshold;
        };

    while (!robot_is_at_angular_velocity(world_ptr))
    {
        std::stringstream ss;
        ss << angular_velocity;
        yield("Robot " + std::to_string(robot_id) +
              " does not have angular_velocity of " + ss.str());
    }
}
