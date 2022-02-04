#include "software/simulated_tests/terminating_validation_functions/robot_state_validation.h"

#include "software/logger/logger.h"

void robotAtPosition(RobotId robot_id, std::shared_ptr<World> world_ptr,
                     const Point& destination, double close_to_destination_threshold,
                     ValidationCoroutine::push_type& yield)
{
    auto robot_is_not_at_destination =
        [robot_id, destination, close_to_destination_threshold](
            std::shared_ptr<World> world_ptr) -> std::optional<Point> {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);
        Robot robot = robot_optional.value();
        if ((robot.position() - destination).length() > close_to_destination_threshold)
        {
            return robot.position();
        }
        else
        {
            return std::nullopt;
        }
    };

    while (auto robot_destination_opt = robot_is_not_at_destination(world_ptr))
    {
        std::stringstream ss;
        ss << "Robot with ID " << robot_id << " is at position "
           << robot_destination_opt.value() << ", expected to be at " << destination;
        yield(ss.str());
    }
}

void robotAtOrientation(RobotId robot_id, std::shared_ptr<World> world_ptr,
                        const Angle& orientation,
                        const Angle& close_to_orientation_threshold,
                        ValidationCoroutine::push_type& yield)
{
    auto robot_is_not_at_orientation =
        [robot_id, orientation, close_to_orientation_threshold](
            std::shared_ptr<World> world_ptr) -> std::optional<Angle> {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);
        Robot robot = robot_optional.value();
        if (robot.orientation().minDiff(orientation) > close_to_orientation_threshold)
        {
            return robot.orientation();
        }
        else
        {
            return std::nullopt;
        }
    };


    while (auto robot_orientation_opt = robot_is_not_at_orientation(world_ptr))
    {
        std::stringstream ss;
        ss << "Robot " << std::to_string(robot_id) << " does not have orientation of "
           << orientation << ", instead it has orientation of "
           << robot_orientation_opt.value();
        yield(ss.str());
    }
}

void robotAtAngularVelocity(RobotId robot_id, std::shared_ptr<World> world_ptr,
                            const AngularVelocity& angular_velocity,
                            const AngularVelocity& close_to_angular_velocity_threshold,
                            ValidationCoroutine::push_type& yield)
{
    auto robot_is_not_at_angular_velocity =
        [robot_id, angular_velocity, close_to_angular_velocity_threshold](
            std::shared_ptr<World> world_ptr) -> std::optional<AngularVelocity> {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        CHECK(robot_optional.has_value())
            << "There is no robot with ID: " + std::to_string(robot_id);

        Robot robot = robot_optional.value();
        if (robot.angularVelocity().minDiff(angular_velocity) >
            close_to_angular_velocity_threshold)
        {
            return robot.angularVelocity();
        }
        else
        {
            return std::nullopt;
        }
    };

    while (auto robot_angular_velocity_opt = robot_is_not_at_angular_velocity(world_ptr))
    {
        std::stringstream ss;
        ss << "Robot " << std::to_string(robot_id)
           << " does not have angular velocity of " << angular_velocity
           << ", instead it has angular velocity of "
           << robot_angular_velocity_opt.value();
        yield(ss.str());
    }
}
