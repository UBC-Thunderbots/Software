#include "software/proto/primitive/primitive_msg_factory.h"

#include "software/logger/logger.h"
#include "software/proto/message_translation/tbots_protobuf.h"

std::unique_ptr<TbotsProto::Primitive> createChipPrimitive(const Point &chip_origin,
                                                           const Angle &chip_direction,
                                                           double chip_distance_meters)
{
    auto chip_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto chip_origin_msg = createPointProto(Point(chip_origin.x(), chip_origin.y()));
    *(chip_primitive_msg->mutable_chip()->mutable_chip_origin()) = *chip_origin_msg;

    auto chip_direction_msg = createAngleProto(chip_direction);
    *(chip_primitive_msg->mutable_chip()->mutable_chip_direction()) = *chip_direction_msg;

    chip_primitive_msg->mutable_chip()->set_chip_distance_meters(
        static_cast<float>(chip_distance_meters));

    return chip_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createKickPrimitive(const Point &kick_origin,
                                                           const Angle &kick_direction,
                                                           double kick_speed_m_per_s)
{
    auto kick_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto kick_origin_msg = createPointProto(Point(kick_origin.x(), kick_origin.y()));
    *(kick_primitive_msg->mutable_kick()->mutable_kick_origin()) = *kick_origin_msg;

    auto kick_direction_msg = createAngleProto(kick_direction);
    *(kick_primitive_msg->mutable_kick()->mutable_kick_direction()) = *kick_direction_msg;

    kick_primitive_msg->mutable_kick()->set_kick_speed_m_per_s(
        static_cast<float>(kick_speed_m_per_s));

    return kick_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_m_per_s, const Angle &final_angle,
    DribblerMode dribbler_mode, AutoChipOrKick auto_chip_or_kick,
    MaxAllowedSpeedMode max_allowed_speed_mode)
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
    return move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createSpinningMovePrimitive(
    const Point &dest, double final_speed_m_per_s,
    const AngularVelocity &angular_velocity, DribblerMode dribbler_mode)
{
    auto spinning_move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto dest_msg             = createPointProto(Point(dest.x(), dest.y()));
    auto angular_velocity_msg = createAngularVelocityProto(angular_velocity);
    *(spinning_move_primitive_msg->mutable_spinning_move()->mutable_angular_velocity()) =
        *angular_velocity_msg;
    *(spinning_move_primitive_msg->mutable_spinning_move()->mutable_destination()) =
        *dest_msg;
    spinning_move_primitive_msg->mutable_spinning_move()->set_final_speed_m_per_s(
        static_cast<float>(final_speed_m_per_s));

    spinning_move_primitive_msg->mutable_spinning_move()->set_dribbler_speed_rpm(
        static_cast<float>(convertDribblerModeToDribblerSpeed(dribbler_mode)));

    return spinning_move_primitive_msg;
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
        default:
            LOG(WARNING) << "MaxAllowedSpeedMode is invalid" << std::endl;
            return 0.0;
    }
}
