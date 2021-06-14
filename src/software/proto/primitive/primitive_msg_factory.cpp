#include "software/proto/primitive/primitive_msg_factory.h"

#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"

std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_m_per_s, const Angle &final_angle,
    DribblerMode dribbler_mode, AutoChipOrKick auto_chip_or_kick,
    MaxAllowedSpeedMode max_allowed_speed_mode, double target_spin_rev_per_s)
{
    auto move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto dest_msg        = createPointProto(Point(dest.x(), dest.y()));
    auto final_angle_msg = createAngleProto(final_angle);
    *(move_primitive_msg->mutable_move()->mutable_final_angle()) = *final_angle_msg;
    *(move_primitive_msg->mutable_move()->mutable_destination()) = *dest_msg;
    move_primitive_msg->mutable_move()->set_final_speed_m_per_s(
        static_cast<float>(final_speed_m_per_s));
    move_primitive_msg->mutable_move()->set_max_speed_m_per_s(static_cast<float>(
        convertMaxAllowedSpeedModeToMaxAllowedSpeed(max_allowed_speed_mode)));

    move_primitive_msg->mutable_move()->set_dribbler_speed_rpm(
        static_cast<float>(convertDribblerModeToDribblerSpeed(dribbler_mode)));

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
    move_primitive_msg->mutable_move()->set_target_spin_rev_per_s(
        static_cast<float>(target_spin_rev_per_s));
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

    return stop_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createEstopPrimitive()
{
    auto estop_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    estop_primitive_msg->mutable_estop();

    return estop_primitive_msg;
}

double convertDribblerModeToDribblerSpeed(DribblerMode dribbler_mode)
{
    switch (dribbler_mode)
    {
        case DribblerMode::INDEFINITE:
            return INDEFINITE_DRIBBLER_SPEED;
        case DribblerMode::MAX_FORCE:
            return MAX_FORCE_DRIBBLER_SPEED;
        case DribblerMode::OFF:
            return 0.0;
        default:
            LOG(WARNING) << "DribblerMode is invalid" << std::endl;
            return 0.0;
    }
}

double convertMaxAllowedSpeedModeToMaxAllowedSpeed(
    MaxAllowedSpeedMode max_allowed_speed_mode)
{
    switch (max_allowed_speed_mode)
    {
        case MaxAllowedSpeedMode::PHYSICAL_LIMIT:
            return ROBOT_MAX_SPEED_METERS_PER_SECOND;
        case MaxAllowedSpeedMode::STOP_COMMAND:
            return STOP_COMMAND_ROBOT_MAX_SPEED_METERS_PER_SECOND;
        case MaxAllowedSpeedMode::TIPTOE:
            return 0.5;
        default:
            LOG(WARNING) << "MaxAllowedSpeedMode is invalid" << std::endl;
            return 0.0;
    }
}
