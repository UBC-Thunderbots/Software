#include "proto/message_translation/ssl_simulation_robot_control.h"

#include "shared/constants.h"
#include "software/geom/angle.h"
#include "software/logger/logger.h"

// Converts rpm and wheel_radius_meters [m] to speed [m/s]
float rpm_to_m_per_s(float rpm, float wheel_radius_meters)
{
    return (2 * (float)M_PI * rpm * wheel_radius_meters) / 60.0f;
}

std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    const TbotsProto::DirectControlPrimitive& direct_control, float front_wheel_angle_deg,
    float back_wheel_angle_deg, float wheel_radius_meters)
{
    switch (direct_control.motor_control().drive_control_case())
    {
        case TbotsProto::MotorControl::kDirectPerWheelControl:
        {
            LOG(FATAL) << "Direct per-wheel control is not supported in simulation";
        }

        case TbotsProto::MotorControl::kDirectVelocityControl:
        {
            auto move_local_velocity = SSLSimulationProto::MoveLocalVelocity();
            move_local_velocity.set_forward(
                static_cast<float>(direct_control.motor_control()
                                       .direct_velocity_control()
                                       .velocity()
                                       .x_component_meters()));
            move_local_velocity.set_left(static_cast<float>(direct_control.motor_control()
                                                                .direct_velocity_control()
                                                                .velocity()
                                                                .y_component_meters()));
            move_local_velocity.set_angular(
                static_cast<float>(direct_control.motor_control()
                                       .direct_velocity_control()
                                       .angular_velocity()
                                       .radians_per_second()));

            auto move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();
            *(move_command->mutable_local_velocity()) = move_local_velocity;
            return move_command;
        }
    }
    return std::make_unique<SSLSimulationProto::RobotMoveCommand>();
}

std::unique_ptr<SSLSimulationProto::RobotCommand> getRobotCommandFromDirectControl(
    unsigned int robot_id,
    std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control,
    RobotConstants_t& robot_constants)
{
    auto move_command = createRobotMoveCommand(
        *direct_control, robot_constants.front_wheel_angle_deg,
        robot_constants.back_wheel_angle_deg, robot_constants.wheel_radius_meters);
    // Values for robot command
    std::optional<float> kick_speed;       // [m/s]
    std::optional<float> kick_angle;       // [degree]
    std::optional<double> dribbler_speed;  // [rpm]

    switch (direct_control->power_control().chicker().chicker_command_case())
    {
        case TbotsProto::PowerControl::ChickerControl::kKickSpeedMPerS:
        {
            kick_speed = direct_control->power_control().chicker().kick_speed_m_per_s();
            kick_angle = std::nullopt;
            break;
        }
        case TbotsProto::PowerControl::ChickerControl::kChipDistanceMeters:
        {
            Angle chip_angle = Angle::fromDegrees(ROBOT_CHIP_ANGLE_DEGREES);
            // Use the formula for the Range of a parabolic projectile
            // Rearrange to solve for the initial velocity.
            // https://courses.lumenlearning.com/boundless-physics/chapter/projectile-motion/
            float range =
                direct_control->power_control().chicker().chip_distance_meters();
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
            switch (direct_control->power_control()
                        .chicker()
                        .auto_chip_or_kick()
                        .auto_chip_or_kick_case())
            {
                case TbotsProto::AutoChipOrKick::kAutokickSpeedMPerS:
                {
                    kick_speed = direct_control->power_control()
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
                    float range = direct_control->power_control()
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
                    direct_control->mutable_power_control()
                        ->mutable_chicker()
                        ->clear_chicker_command();
                    break;
                }
            }
            break;
        }
        case TbotsProto::PowerControl::ChickerControl::CHICKER_COMMAND_NOT_SET:
        {
            direct_control->mutable_power_control()
                ->mutable_chicker()
                ->clear_chicker_command();
            break;
        }
    }

    return createRobotCommand(robot_id, std::move(move_command), kick_speed, kick_angle,
                              direct_control->motor_control().dribbler_speed_rpm());
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
        // NOTE: our dribbler speed for the robots is negative RPM to dribble, but
        // the RobotCommand expects positive RPM to dribble. So we invert the sign
        robot_command->set_dribbler_speed(-1.0f *
                                          static_cast<float>(dribbler_speed.value()));
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
