#include "proto/primitive/primitive_msg_factory.h"

#include "proto/message_translation/tbots_protobuf.h"
#include "software/logger/logger.h"

std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const TbotsProto::MotionControl& motion_control, const Angle& final_angle,
    double final_speed, const TbotsProto::DribblerMode& dribbler_mode,
    const TbotsProto::BallCollisionType& ball_collision_type,
    const AutoChipOrKick& auto_chip_or_kick,
    const TbotsProto::MaxAllowedSpeedMode& max_allowed_speed_mode,
    double target_spin_rev_per_s, const RobotConstants_t& robot_constants,
    std::optional<double> cost_override)
{
    auto move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto cost = motion_control.normalized_path_length();
    if (cost_override.has_value())
    {
        cost = cost_override.value();
    }

    *(move_primitive_msg->mutable_move()->mutable_motion_control()) = motion_control;
    move_primitive_msg->mutable_move()->set_final_speed_m_per_s(
        static_cast<float>(final_speed));
    move_primitive_msg->mutable_move()->set_max_speed_m_per_s(
        static_cast<float>(convertMaxAllowedSpeedModeToMaxAllowedSpeed(
            max_allowed_speed_mode, robot_constants)));

    *(move_primitive_msg->mutable_move()->mutable_final_angle()) =
        *createAngleProto(final_angle);
    move_primitive_msg->mutable_move()->set_dribbler_speed_rpm(static_cast<float>(
        convertDribblerModeToDribblerSpeed(dribbler_mode, robot_constants)));

    if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOCHIP)
    {
        move_primitive_msg->mutable_move()
            ->mutable_auto_chip_or_kick()
            ->set_autochip_distance_meters(
                static_cast<float>(auto_chip_or_kick.autochip_distance_m));
    }
    else if (auto_chip_or_kick.auto_chip_kick_mode == AutoChipOrKickMode::AUTOKICK)
    {
        move_primitive_msg->mutable_move()
            ->mutable_auto_chip_or_kick()
            ->set_autokick_speed_m_per_s(
                static_cast<float>(auto_chip_or_kick.autokick_speed_m_per_s));
    }

    move_primitive_msg->mutable_move()->set_ball_collision_type(ball_collision_type);

    move_primitive_msg->mutable_move()->set_target_spin_rev_per_s(
        static_cast<float>(target_spin_rev_per_s));
    move_primitive_msg->set_cost(cost);
    return move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createStopPrimitive(bool coast)
{
    auto stop_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    if (coast)
    {
        stop_primitive_msg->mutable_stop()->set_stop_type(
            TbotsProto::StopPrimitive::COAST);
    }
    else
    {
        stop_primitive_msg->mutable_stop()->set_stop_type(
            TbotsProto::StopPrimitive::BRAKE);
    }

    stop_primitive_msg->set_cost(1.0);

    return stop_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createEstopPrimitive()
{
    auto estop_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    estop_primitive_msg->mutable_estop();

    return estop_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createDirectControlPrimitive(
    const Vector& velocity, AngularVelocity angular_velocity, double dribbler_speed_rpm,
    const TbotsProto::AutoChipOrKick& auto_chip_or_kick)
{
    auto direct_control_primitive_msg = std::make_unique<TbotsProto::Primitive>();
    auto direct_velocity_control =
        std::make_unique<TbotsProto::MotorControl::DirectVelocityControl>();

    *(direct_velocity_control->mutable_velocity()) = *createVectorProto(velocity);
    *(direct_velocity_control->mutable_angular_velocity()) =
        *createAngularVelocityProto(angular_velocity);

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_motor_control()
          ->mutable_direct_velocity_control()) = *direct_velocity_control;

    direct_control_primitive_msg->mutable_direct_control()
        ->mutable_motor_control()
        ->set_dribbler_speed_rpm(static_cast<float>(dribbler_speed_rpm));

    *(direct_control_primitive_msg->mutable_direct_control()
          ->mutable_power_control()
          ->mutable_chicker()
          ->mutable_auto_chip_or_kick()) = auto_chip_or_kick;
    return direct_control_primitive_msg;
}

double convertDribblerModeToDribblerSpeed(TbotsProto::DribblerMode dribbler_mode,
                                          RobotConstants_t robot_constants)
{
    switch (dribbler_mode)
    {
        case TbotsProto::DribblerMode::INDEFINITE:
            return robot_constants.indefinite_dribbler_speed_rpm;
        case TbotsProto::DribblerMode::MAX_FORCE:
            return robot_constants.max_force_dribbler_speed_rpm;
        case TbotsProto::DribblerMode::OFF:
            return 0.0;
        default:
            LOG(WARNING) << "DribblerMode is invalid" << std::endl;
            return 0.0;
    }
}

double convertMaxAllowedSpeedModeToMaxAllowedSpeed(
    TbotsProto::MaxAllowedSpeedMode max_allowed_speed_mode,
    RobotConstants_t robot_constants)
{
    switch (max_allowed_speed_mode)
    {
        case TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT:
            return robot_constants.robot_max_speed_m_per_s;
        case TbotsProto::MaxAllowedSpeedMode::STOP_COMMAND:
            return STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND;
        case TbotsProto::MaxAllowedSpeedMode::COLLISIONS_ALLOWED:
            return COLLISION_ALLOWED_ROBOT_MAX_SPEED_METERS_PER_SECOND;
        case TbotsProto::MaxAllowedSpeedMode::DRIBBLE_GET_POSSESSION:
            return 0.5;
        case TbotsProto::MaxAllowedSpeedMode::DRIBBLE_DRIBBLING:
            return 1.5;
        default:
            LOG(WARNING) << "MaxAllowedSpeedMode is invalid" << std::endl;
            return 0.0;
    }
}
