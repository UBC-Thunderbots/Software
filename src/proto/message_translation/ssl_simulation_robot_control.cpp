#include "proto/message_translation/ssl_simulation_robot_control.h"

#include "shared/constants.h"
#include "software/geom/angle.h"

extern "C"
{
#include "firmware/shared/physics.h"
}

// Converts rpm and wheel_radius_meters [m] to speed [m/s]
float rpm_to_m_per_s(float rpm, float wheel_radius_meters)
{
    return (2 * (float)M_PI * rpm * wheel_radius_meters) / 60.0f;
}

std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    double wheel_rpm_front_right, double wheel_rpm_front_left, double wheel_rpm_back_left,
    double wheel_rpm_back_right, float front_wheel_angle_deg, float back_wheel_angle_deg,
    float wheel_radius_meters)
{
    auto move_local_velocity = SSLSimulationProto::MoveLocalVelocity();

    // Convert the units of wheel speeds to m/s
    float front_left_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_front_left), wheel_radius_meters);
    float back_left_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_back_left), wheel_radius_meters);
    float back_right_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_back_right), wheel_radius_meters);
    float front_right_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_front_right), wheel_radius_meters);
    float wheel_speeds[4]{front_left_m_per_s, back_left_m_per_s, back_right_m_per_s,
                          front_right_m_per_s};
    float robot_local_speed[3]{0.0, 0.0, 0.0};
    shared_physics_speed4ToSpeed3(wheel_speeds, robot_local_speed, front_wheel_angle_deg,
                                  back_wheel_angle_deg);

    // Convert speed [m/s] to angular velocity [rad/s]
    robot_local_speed[2] =
        robot_local_speed[2] / static_cast<float>(ROBOT_MAX_RADIUS_METERS);

    move_local_velocity.set_forward(robot_local_speed[0]);
    move_local_velocity.set_left(robot_local_speed[1]);
    move_local_velocity.set_angular(robot_local_speed[2]);

    auto move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();

    *(move_command->mutable_local_velocity()) = move_local_velocity;

    return move_command;
}

std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    const TbotsProto::DirectControlPrimitive& direct_control, float front_wheel_angle_deg,
    float back_wheel_angle_deg, float wheel_radius_meters)
{
    switch (direct_control.wheel_control_case())
    {
        case TbotsProto::DirectControlPrimitive::kDirectPerWheelControl:
        {
            return createRobotMoveCommand(
                direct_control.direct_per_wheel_control().front_left_wheel_rpm(),
                direct_control.direct_per_wheel_control().back_left_wheel_rpm(),
                direct_control.direct_per_wheel_control().front_right_wheel_rpm(),
                direct_control.direct_per_wheel_control().back_right_wheel_rpm(),
                front_wheel_angle_deg, back_wheel_angle_deg, wheel_radius_meters);
        }

        case TbotsProto::DirectControlPrimitive::kDirectVelocityControl:
        {
            auto move_local_velocity = SSLSimulationProto::MoveLocalVelocity();
            move_local_velocity.set_forward(
                static_cast<float>(direct_control.direct_velocity_control()
                                       .velocity()
                                       .x_component_meters()));
            move_local_velocity.set_left(
                static_cast<float>(direct_control.direct_velocity_control()
                                       .velocity()
                                       .y_component_meters()));
            move_local_velocity.set_angular(
                static_cast<float>(direct_control.direct_velocity_control()
                                       .angular_velocity()
                                       .radians_per_second()));

            auto move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();
            *(move_command->mutable_local_velocity()) = move_local_velocity;
            return move_command;
        }
        case TbotsProto::DirectControlPrimitive::WHEEL_CONTROL_NOT_SET:
        {
            return std::make_unique<SSLSimulationProto::RobotMoveCommand>();
        }
    }
    return std::make_unique<SSLSimulationProto::RobotMoveCommand>();
}

std::unique_ptr<SSLSimulationProto::RobotCommand> getRobotCommandFromDirectControl(
    unsigned int robot_id,
    std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control,
    RobotConstants_t& robot_constants, WheelConstants_t wheel_constants)
{
    auto move_command = createRobotMoveCommand(
        *direct_control, robot_constants.front_wheel_angle_deg,
        robot_constants.back_wheel_angle_deg, wheel_constants.wheel_radius_meters);
    // Values for robot command
    std::optional<float> kick_speed;       // [m/s]
    std::optional<float> kick_angle;       // [degree]
    std::optional<double> dribbler_speed;  // [rpm]

    switch (direct_control->power().chicker().chicker_command_case())
    {
        case TbotsProto::PowerControl::ChickerControl::kKickSpeedMPerS:
        {
            kick_speed = direct_control->power().chicker().kick_speed_m_per_s();
            kick_angle = std::nullopt;
            break;
        }
        case TbotsProto::PowerControl::ChickerControl::kChipDistanceMeters:
        {
            Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);
            // Use the formula for the Range of a parabolic projectile
            // Rearrange to solve for the initial velocity.
            // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
            float range = direct_control->power().chicker().chip_distance_meters();
            float numerator =
                range *
                static_cast<float>(ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED);
            float denominator = static_cast<float>(2.0f * (chip_angle * 2.0f).sin());
            float chip_speed  = static_cast<float>(std::sqrt(numerator / denominator));

            kick_speed = chip_speed;
            kick_angle = chip_angle.toDegrees();
            break;
        }
        case TbotsProto::PowerControl::ChickerControl::kAutoChipOrKick:
        {
            switch (direct_control->power()
                        .chicker()
                        .auto_chip_or_kick()
                        .auto_chip_or_kick_case())
            {
                case TbotsProto::AutoChipOrKick::kAutokickSpeedMPerS:
                {
                    kick_speed = direct_control->power()
                                     .chicker()
                                     .auto_chip_or_kick()
                                     .autokick_speed_m_per_s();
                    kick_angle = std::nullopt;
                    break;
                }
                case TbotsProto::AutoChipOrKick::kAutochipDistanceMeters:
                {
                    Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);
                    // Use the formula for the Range of a parabolic projectile
                    // Rearrange to solve for the initial velocity.
                    // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
                    float range = direct_control->power()
                                      .chicker()
                                      .auto_chip_or_kick()
                                      .autochip_distance_meters();
                    float numerator =
                        range *
                        static_cast<float>(
                            ACCELERATION_DUE_TO_GRAVITY_METERS_PER_SECOND_SQUARED);
                    float denominator =
                        static_cast<float>(2.0f * (chip_angle * 2.0f).sin());
                    float chip_speed =
                        static_cast<float>(std::sqrt(numerator / denominator));

                    kick_speed = chip_speed;
                    kick_angle = chip_angle.toDegrees();
                    break;
                }
                case TbotsProto::AutoChipOrKick::AUTO_CHIP_OR_KICK_NOT_SET:
                {
                    direct_control->mutable_power()
                        ->mutable_chicker()
                        ->clear_chicker_command();
                    break;
                }
            }
            break;
        }
        case TbotsProto::PowerControl::ChickerControl::CHICKER_COMMAND_NOT_SET:
        {
            direct_control->mutable_power()->mutable_chicker()->clear_chicker_command();
            break;
        }
    }

    return createRobotCommand(robot_id, std::move(move_command), kick_speed, kick_angle,
                              direct_control->dribbler_speed_rpm());
}

std::unique_ptr<SSLSimulationProto::RobotCommand> createRobotCommand(
    unsigned int robot_id,
    std::unique_ptr<SSLSimulationProto::RobotMoveCommand> move_command,
    std::optional<double> kick_speed, std::optional<double> kick_angle,
    std::optional<double> dribbler_speed)
{
    auto robot_command = std::make_unique<SSLSimulationProto::RobotCommand>();

    robot_command->set_id(robot_id);

    if (kick_speed.has_value())
    {
        robot_command->set_kick_speed(static_cast<float>(kick_speed.value()));
    }
    if (kick_angle.has_value())
    {
        robot_command->set_kick_angle(static_cast<float>(kick_angle.value()));
    }
    if (dribbler_speed.has_value())
    {
        robot_command->set_dribbler_speed(static_cast<float>(dribbler_speed.value()));
    }

    *(robot_command->mutable_move_command()) = *move_command;

    return robot_command;
}

std::unique_ptr<SSLSimulationProto::RobotControl> createRobotControl(
    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands)
{
    auto robot_control = std::make_unique<SSLSimulationProto::RobotControl>();

    for (auto&& robot_command : robot_commands)
    {
        *(robot_control->add_robot_commands()) = *robot_command;
    }

    return robot_control;
}
