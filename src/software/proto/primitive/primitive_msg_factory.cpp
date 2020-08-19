#include "software/proto/primitive/primitive_msg_factory.h"

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

std::unique_ptr<TbotsProto::Primitive> createKickPrimitive(
    const Point &kick_origin, const Angle &kick_direction,
    double kick_speed_meters_per_second)
{
    auto kick_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto kick_origin_msg = createPointProto(Point(kick_origin.x(), kick_origin.y()));
    *(kick_primitive_msg->mutable_kick()->mutable_kick_origin()) = *kick_origin_msg;

    auto kick_direction_msg = createAngleProto(kick_direction);
    *(kick_primitive_msg->mutable_kick()->mutable_kick_direction()) = *kick_direction_msg;

    kick_primitive_msg->mutable_kick()->set_kick_speed_meters_per_second(
        static_cast<float>(kick_speed_meters_per_second));

    return kick_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, bool slow,
    const Angle &final_angle, double dribbler_speed_rpm)
{
    auto move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto position_params_msg = std::make_unique<TbotsProto::MovePositionParams>();
    auto dest_msg            = createPointProto(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(
        static_cast<float>(final_speed_meters_per_second));
    position_params_msg->set_slow(slow);
    *(move_primitive_msg->mutable_move()->mutable_position_params()) =
        *position_params_msg;

    auto final_angle_msg = createAngleProto(final_angle);
    *(move_primitive_msg->mutable_move()->mutable_final_angle()) = *final_angle_msg;

    move_primitive_msg->mutable_move()->set_dribbler_speed_rpm(
        static_cast<float>(dribbler_speed_rpm));

    return move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createLegacyMovePrimitive(
    const Point &dest, const Angle &final_angle, double final_speed,
    DribblerEnable enable_dribbler, MoveType move_type, AutochickType autokick)
{
    return createMovePrimitive(dest, final_speed, move_type == MoveType::SLOW,
                               final_angle,
                               enable_dribbler == DribblerEnable::ON ? 16000 : 0);
}

std::unique_ptr<TbotsProto::Primitive> createSpinningMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, bool slow,
    const AngularVelocity &angular_velocity, double dribbler_speed_rpm)
{
    auto spinning_move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto position_params_msg = std::make_unique<TbotsProto::MovePositionParams>();
    auto dest_msg            = createPointProto(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(
        static_cast<float>(final_speed_meters_per_second));
    position_params_msg->set_slow(slow);
    *(spinning_move_primitive_msg->mutable_spinning_move()->mutable_position_params()) =
        *position_params_msg;

    auto angular_velocity_msg = createAngularVelocityProto(angular_velocity);
    *(spinning_move_primitive_msg->mutable_spinning_move()->mutable_angular_velocity()) =
        *angular_velocity_msg;

    spinning_move_primitive_msg->mutable_spinning_move()->set_dribbler_speed_rpm(
        static_cast<float>(dribbler_speed_rpm));

    return spinning_move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createAutochipMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, bool slow,
    const Angle &final_angle, double dribbler_speed_rpm, double chip_distance_meters)
{
    auto autochip_move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto position_params_msg = std::make_unique<TbotsProto::MovePositionParams>();
    auto dest_msg            = createPointProto(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(
        static_cast<float>(final_speed_meters_per_second));
    position_params_msg->set_slow(slow);
    *(autochip_move_primitive_msg->mutable_autochip_move()->mutable_position_params()) =
        *position_params_msg;

    auto final_angle_msg = createAngleProto(final_angle);
    *(autochip_move_primitive_msg->mutable_autochip_move()->mutable_final_angle()) =
        *final_angle_msg;

    autochip_move_primitive_msg->mutable_autochip_move()->set_dribbler_speed_rpm(
        static_cast<float>(dribbler_speed_rpm));

    autochip_move_primitive_msg->mutable_autochip_move()->set_chip_distance_meters(
        static_cast<float>(chip_distance_meters));

    return autochip_move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createAutokickMovePrimitive(
    const Point &dest, double final_speed_meters_per_second, bool slow,
    const Angle &final_angle, double dribbler_speed_rpm,
    double kick_speed_meters_per_second)
{
    auto autokick_move_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    auto position_params_msg = std::make_unique<TbotsProto::MovePositionParams>();
    auto dest_msg            = createPointProto(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(
        static_cast<float>(final_speed_meters_per_second));
    position_params_msg->set_slow(slow);
    *(autokick_move_primitive_msg->mutable_autokick_move()->mutable_position_params()) =
        *position_params_msg;

    auto final_angle_msg = createAngleProto(final_angle);
    *(autokick_move_primitive_msg->mutable_autokick_move()->mutable_final_angle()) =
        *final_angle_msg;

    autokick_move_primitive_msg->mutable_autokick_move()->set_dribbler_speed_rpm(
        static_cast<float>(dribbler_speed_rpm));

    autokick_move_primitive_msg->mutable_autokick_move()
        ->set_kick_speed_meters_per_second(
            static_cast<float>(kick_speed_meters_per_second));

    return autokick_move_primitive_msg;
}

std::unique_ptr<TbotsProto::Primitive> createStopPrimitive(bool coast)
{
    auto stop_primitive_msg = std::make_unique<TbotsProto::Primitive>();

    if (coast)
    {
        stop_primitive_msg->mutable_stop()->set_stop_type(
            stop_primitive_msg->mutable_stop()->COAST);
    }
    else
    {
        stop_primitive_msg->mutable_stop()->set_stop_type(
            stop_primitive_msg->mutable_stop()->BRAKE);
    }

    return stop_primitive_msg;
}
