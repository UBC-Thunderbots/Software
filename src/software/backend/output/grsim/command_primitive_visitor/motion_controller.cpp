/**
 * .cpp file for the grSim motion controller.
 *
 * In the current state it is a bang-bang controller.
 *
 * It assumed the robot max acceleration is constant.
 *
 * Uses constant acceleration kinematics equations to
 * calculate changes in speed.
 *
 * See https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control for more info
 */

#include "software/backend/output/grsim/command_primitive_visitor/motion_controller.h"

#include <algorithm>

#include "software/logger/logger.h"

// Creates a struct which inherits all lambda function given to it and uses their
// Ts::operator(). This can be passed to std::visit to easily write multiple different
// lambdas for each type of motion controller commands below. See
// https://en.cppreference.com/w/cpp/utility/variant/visit for more details.
template <class... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overload(Ts...)->overload<Ts...>;

using MotionControllerCommand =
    std::variant<MotionController::PositionCommand, MotionController::VelocityCommand>;


MotionController::Velocity MotionController::bangBangVelocityController(
    Robot robot, const double delta_time, MotionControllerCommand motion_command)
{
    MotionController::Velocity robot_velocities;

    if (delta_time < 0)
    {
        throw std::invalid_argument(
            "GrSim Motion controller received a negative delta time");
    }
    else if (delta_time == 0)
    {
        robot_velocities.linear_velocity  = robot.velocity();
        robot_velocities.angular_velocity = robot.angularVelocity();
    }
    else
    {
        std::visit(
            overload{
                [&robot_velocities, robot, delta_time,
                 this](MotionController::PositionCommand& command) {
                    robot_velocities.linear_velocity =
                        MotionController::determineLinearVelocityFromPosition(
                            robot, command.global_destination,
                            command.final_speed_at_destination, delta_time,
                            this->max_speed_meters_per_second,
                            this->max_angular_speed_radians_per_second);
                    robot_velocities.angular_velocity =
                        MotionController::determineAngularVelocityFromPosition(
                            robot, command.final_orientation, delta_time,
                            this->max_angular_speed_radians_per_second,
                            this->max_angular_acceleration_meters_per_second_squared);
                },

                [&robot_velocities, robot, delta_time,
                 this](MotionController::VelocityCommand& command) {
                    robot_velocities.linear_velocity =
                        MotionController::determineLinearVelocityFromVelocity(
                            robot, command.linear_velocity,
                            this->max_acceleration_meters_per_second_squared, delta_time,
                            max_speed_meters_per_second);
                    robot_velocities.angular_velocity =
                        MotionController::determineAngularVelocityFromVelocity(
                            robot, command.angular_velocity,
                            this->max_angular_acceleration_meters_per_second_squared,
                            delta_time);
                },

                [](auto arg) {
                    throw std::invalid_argument(
                        "Unrecognized type passed via std::variant");
                }},
            motion_command);
    }

    return robot_velocities;
}

AngularVelocity MotionController::determineAngularVelocityFromPosition(
    const Robot robot, const Angle desired_final_orientation, const double delta_time,
    const double max_angular_speed_radians_per_second,
    const double max_angular_acceleration_radians_per_second_squared)
{
    // Calculate the angular difference between us and a goal
    Angle angle_difference = (desired_final_orientation - robot.orientation()).clamp();

    // Calculate our desired additional turn rate based on a sqrt-esque profile
    // This allows us to rapidly bring the velocity to zero when we're near the
    // target angle
    double new_robot_angular_velocity_magnitude =
        std::sqrt(std::max<double>(
            (angle_difference.abs() - Angle::fromDegrees(0.2)).toRadians(), 0)) *
        3;
    // Cap our angular velocity at the robot's physical limit
    new_robot_angular_velocity_magnitude = std::min<double>(
        new_robot_angular_velocity_magnitude, max_angular_speed_radians_per_second);
    // Figure out the maximum acceleration the robot is physically capable of
    double max_additional_angular_velocity =
        max_angular_acceleration_radians_per_second_squared * delta_time;

    // Compute the final velocity by taking the minimum of the physical max additional
    // turn rate and our desired additional turn rate
    double new_angular_velocity =
        robot.angularVelocity().toRadians() +
        std::copysign(max_additional_angular_velocity, angle_difference.toRadians());
    new_angular_velocity =
        std::clamp<double>(new_angular_velocity, -new_robot_angular_velocity_magnitude,
                           new_robot_angular_velocity_magnitude);

    return AngularVelocity::fromRadians(new_angular_velocity);
}

Vector MotionController::determineLinearVelocityFromPosition(
    const Robot robot, const Point dest, const double desired_final_speed,
    const double delta_time, const double max_speed_meters_per_second,
    const double max_acceleration_meters_per_second_squared)
{
    // Figure out the maximum (physically possible) velocity we can add to the robot
    double max_magnitude_of_velocity_to_apply =
        max_acceleration_meters_per_second_squared * delta_time;

    // Calculate a unit vector from the robot to the destination
    Vector unit_vector_to_dest = (dest - robot.position()).normalize();

    // Calculate the vector of our velocity perpendicular to the vector toward the
    // destination
    Vector robot_velocity_perpendicular_to_dest =
        robot.velocity().dot(unit_vector_to_dest.perpendicular()) *
        unit_vector_to_dest.perpendicular();

    // Use the "remaining" velocity (based on physical limits) to move us along the vector
    // towards the destination
    double magnitude_additional_velocity_towards_dest =
        std::max<double>(0, max_magnitude_of_velocity_to_apply -
                                robot_velocity_perpendicular_to_dest.length());
    Vector additional_velocity_towards_dest =
        magnitude_additional_velocity_towards_dest * unit_vector_to_dest;

    // The resulting additional velocity to apply to the robot is the velocity required to
    // counteract our movement perpendicular to the vector towards the destination, with
    // the "remaining" velocity pushing us along the vector to the destination
    Vector additional_velocity =
        -robot_velocity_perpendicular_to_dest + additional_velocity_towards_dest;

    // Clamp the new robot velocity to the minimum of the physical maximum velocity and
    // a square root-esque function of the distance to the destination. This allows us to
    // bring the robot velocity to zero as we approach the destination
    double new_robot_velocity_magnitude =
        std::sqrt(std::max<double>((dest - robot.position()).length() - 0.01, 0)) * 2 +
        desired_final_speed;

    // https://github.com/UBC-Thunderbots/Software/issues/270
    // Check for the case where we are moving away from the target AND the additional
    // velocity is greater in magnitude than the current velocity, as the above
    // function will increase in magnitude as the distance to the destination increases

    bool moving_away_from_dest =
        (robot.velocity().orientation() - unit_vector_to_dest.orientation())
            .clamp()
            .abs() > Angle::quarter();

    if (moving_away_from_dest && additional_velocity.length() < robot.velocity().length())
    {
        new_robot_velocity_magnitude *= -1;
    }
    new_robot_velocity_magnitude =
        std::clamp<double>(new_robot_velocity_magnitude, -max_speed_meters_per_second,
                           max_speed_meters_per_second);

    Vector new_robot_velocity = new_robot_velocity_magnitude *
                                (robot.velocity() + additional_velocity).normalize();

    // Translate velocities into robot coordinates
    Vector new_robot_velocity_in_robot_coordinates =
        new_robot_velocity.rotate(-robot.orientation());

    return new_robot_velocity_in_robot_coordinates;
}

Vector MotionController::determineLinearVelocityFromVelocity(
    const Robot robot, const Vector linear_velocity,
    const double max_acceleration_meters_per_second_squared, const double delta_time,
    const double max_speed_meters_per_second)
{
    double velocity_to_add = max_acceleration_meters_per_second_squared * delta_time;
    double new_velocity_x, new_velocity_y;

    if (robot.velocity().x() < linear_velocity.x())
    {
        new_velocity_x = robot.velocity().x() + velocity_to_add;
    }
    else
    {
        new_velocity_x = robot.velocity().x() - velocity_to_add;
    }

    if (robot.velocity().y() < linear_velocity.y())
    {
        new_velocity_y = robot.velocity().y() + velocity_to_add;
    }
    else
    {
        new_velocity_y = robot.velocity().y() - velocity_to_add;
    }

    Vector new_velocity_norm      = Vector(new_velocity_x, new_velocity_y).normalize();
    double new_velocity_magnitude = Vector(new_velocity_x, new_velocity_y).length();

    new_velocity_magnitude = std::clamp<double>(
        new_velocity_magnitude, -linear_velocity.length(), linear_velocity.length());
    new_velocity_magnitude =
        std::clamp<double>(new_velocity_magnitude, -max_speed_meters_per_second,
                           max_speed_meters_per_second);
    return new_velocity_norm * new_velocity_magnitude;
}

AngularVelocity MotionController::determineAngularVelocityFromVelocity(
    const Robot robot, AngularVelocity angular_velocity,
    const double max_angular_acceleration_meters_per_second_squared,
    const double delta_time)
{
    double velocity_to_add =
        max_angular_acceleration_meters_per_second_squared * delta_time;
    double new_angular_velocity;

    if (robot.angularVelocity() < angular_velocity)
    {
        new_angular_velocity = robot.angularVelocity().toRadians() + velocity_to_add;
    }
    else
    {
        new_angular_velocity = robot.angularVelocity().toRadians() - velocity_to_add;
    }

    new_angular_velocity =
        std::clamp(new_angular_velocity, -std::abs(angular_velocity.toRadians()),
                   std::abs(angular_velocity.toRadians()));
    new_angular_velocity = std::clamp(new_angular_velocity,
                                      -max_angular_acceleration_meters_per_second_squared,
                                      max_angular_acceleration_meters_per_second_squared);
    return AngularVelocity::fromRadians(new_angular_velocity);
}
