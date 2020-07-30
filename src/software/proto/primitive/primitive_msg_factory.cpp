#include "software/proto/primitive/primitive_msg_factory.h"

#include "software/proto/message_translation/tbots_protobuf.h"

std::unique_ptr<ChipPrimitiveMsg> createChipPrimitiveMsg(const Point &chip_origin,
                                                         const Angle &chip_direction,
                                                         float chip_distance_meters)
{
    auto chip_primitive_msg = std::make_unique<ChipPrimitiveMsg>();

    auto chip_origin_msg = createPointMsg(Point(chip_origin.x(), chip_origin.y()));
    *(chip_primitive_msg->mutable_chip_origin()) = *chip_origin_msg;

    auto chip_direction_msg                         = createAngleMsg(chip_direction);
    *(chip_primitive_msg->mutable_chip_direction()) = *chip_direction_msg;

    chip_primitive_msg->set_chip_distance_meters(chip_distance_meters);

    return chip_primitive_msg;
}

std::unique_ptr<KickPrimitiveMsg> createKickPrimitiveMsg(
    const Point &kick_origin, const Angle &kick_direction,
    float kick_speed_meters_per_second)
{
    auto kick_primitive_msg = std::make_unique<KickPrimitiveMsg>();

    auto kick_origin_msg = createPointMsg(Point(kick_origin.x(), kick_origin.y()));
    *(kick_primitive_msg->mutable_kick_origin()) = *kick_origin_msg;

    auto kick_direction_msg                         = createAngleMsg(kick_direction);
    *(kick_primitive_msg->mutable_kick_direction()) = *kick_direction_msg;

    kick_primitive_msg->set_kick_speed_meters_per_second(kick_speed_meters_per_second);

    return kick_primitive_msg;
}

std::unique_ptr<MovePrimitiveMsg> createMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm)
{
    auto move_primitive_msg = std::make_unique<MovePrimitiveMsg>();

    auto position_params_msg = std::make_unique<MovePositionParams>();
    auto dest_msg            = createPointMsg(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(final_speed_meters_per_second);
    position_params_msg->set_slow(slow);
    *(move_primitive_msg->mutable_position_params()) = *position_params_msg;

    auto final_angle_msg                         = createAngleMsg(final_angle);
    *(move_primitive_msg->mutable_final_angle()) = *final_angle_msg;

    move_primitive_msg->set_dribbler_speed_rpm(dribbler_speed_rpm);

    return move_primitive_msg;
}

std::unique_ptr<SpinningMovePrimitiveMsg> createSpinningMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const AngularVelocity &angular_velocity, float dribbler_speed_rpm)
{
    auto spinning_move_primitive_msg = std::make_unique<SpinningMovePrimitiveMsg>();

    auto position_params_msg = std::make_unique<MovePositionParams>();
    auto dest_msg            = createPointMsg(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(final_speed_meters_per_second);
    position_params_msg->set_slow(slow);
    *(spinning_move_primitive_msg->mutable_position_params()) = *position_params_msg;

    auto angular_velocity_msg = createAngularVelocityMsg(angular_velocity);
    *(spinning_move_primitive_msg->mutable_angular_velocity()) = *angular_velocity_msg;

    spinning_move_primitive_msg->set_dribbler_speed_rpm(dribbler_speed_rpm);

    return spinning_move_primitive_msg;
}

std::unique_ptr<AutochipMovePrimitiveMsg> createAutochipMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm, float chip_distance_meters)
{
    auto autochip_move_primitive_msg = std::make_unique<AutochipMovePrimitiveMsg>();

    auto position_params_msg = std::make_unique<MovePositionParams>();
    auto dest_msg            = createPointMsg(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(final_speed_meters_per_second);
    position_params_msg->set_slow(slow);
    *(autochip_move_primitive_msg->mutable_position_params()) = *position_params_msg;

    auto final_angle_msg                                  = createAngleMsg(final_angle);
    *(autochip_move_primitive_msg->mutable_final_angle()) = *final_angle_msg;

    autochip_move_primitive_msg->set_dribbler_speed_rpm(dribbler_speed_rpm);

    autochip_move_primitive_msg->set_chip_distance_meters(chip_distance_meters);

    return autochip_move_primitive_msg;
}

std::unique_ptr<AutokickMovePrimitiveMsg> createAutokickMovePrimitiveMsg(
    const Point &dest, float final_speed_meters_per_second, bool slow,
    const Angle &final_angle, float dribbler_speed_rpm,
    float kick_speed_meters_per_second)
{
    auto autokick_move_primitive_msg = std::make_unique<AutokickMovePrimitiveMsg>();

    auto position_params_msg = std::make_unique<MovePositionParams>();
    auto dest_msg            = createPointMsg(Point(dest.x(), dest.y()));
    *(position_params_msg->mutable_destination()) = *dest_msg;
    position_params_msg->set_final_speed_meters_per_second(final_speed_meters_per_second);
    position_params_msg->set_slow(slow);
    *(autokick_move_primitive_msg->mutable_position_params()) = *position_params_msg;

    auto final_angle_msg                                  = createAngleMsg(final_angle);
    *(autokick_move_primitive_msg->mutable_final_angle()) = *final_angle_msg;

    autokick_move_primitive_msg->set_dribbler_speed_rpm(dribbler_speed_rpm);

    autokick_move_primitive_msg->set_kick_speed_meters_per_second(
        kick_speed_meters_per_second);

    return autokick_move_primitive_msg;
}

std::unique_ptr<StopPrimitiveMsg> createStopPrimitiveMsg(StopType stop_type)
{
    auto stop_primitive_msg = std::make_unique<StopPrimitiveMsg>();

    if (stop_type == BRAKE)
    {
        stop_primitive_msg->set_stop_type(stop_primitive_msg->BRAKE);
    }
    else
    {
        stop_primitive_msg->set_stop_type(stop_primitive_msg->COAST);
    }

    return stop_primitive_msg;
}
