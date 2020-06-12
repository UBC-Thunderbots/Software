#include "software/proto/message_translation/protobuf_primitive_visitor.h"

#include "shared/constants.h"
#include "shared/proto/radio_primitive.pb.h"
#include "software/primitive/all_primitives.h"

std::unique_ptr<RadioPrimitiveMsg> ProtobufPrimitiveVisitor::getRadioPrimitiveMsg(
    const Primitive &primitive)
{
    primitive.accept(*this);
    return std::move(prim_msg_ptr);
}

void ProtobufPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::CATCH);
    prim_msg_ptr->set_parameter1(catch_primitive.getVelocity());
    prim_msg_ptr->set_parameter2(catch_primitive.getDribblerSpeed());
    prim_msg_ptr->set_parameter3(catch_primitive.getMargin());
    prim_msg_ptr->set_extra_bits(0);
}

void ProtobufPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::SHOOT);
    prim_msg_ptr->set_parameter1(chip_primitive.getChipOrigin().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(chip_primitive.getChipOrigin().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(chip_primitive.getChipDirection().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_parameter4(chip_primitive.getChipDistance() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_extra_bits(static_cast<uint32_t>(2 | 1));
}

void ProtobufPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::DIRECT_VELOCITY);
    prim_msg_ptr->set_parameter1(direct_velocity_primitive.getXVelocity() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(direct_velocity_primitive.getYVelocity() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(direct_velocity_primitive.getAngularVelocity() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_extra_bits(direct_velocity_primitive.getDribblerRpm() *
                                 DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void ProtobufPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::DIRECT_WHEELS);
    prim_msg_ptr->set_parameter1(
        static_cast<double>(direct_wheels_primitive.getWheel0Power()));
    prim_msg_ptr->set_parameter2(
        static_cast<double>(direct_wheels_primitive.getWheel1Power()));
    prim_msg_ptr->set_parameter3(
        static_cast<double>(direct_wheels_primitive.getWheel2Power()));
    prim_msg_ptr->set_parameter4(
        static_cast<double>(direct_wheels_primitive.getWheel3Power()));
    prim_msg_ptr->set_extra_bits(direct_wheels_primitive.getDribblerRPM() *
                                 DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void ProtobufPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::DRIBBLE);
    prim_msg_ptr->set_parameter1(dribble_primitive.getDestination().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(dribble_primitive.getDestination().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(dribble_primitive.getFinalAngle().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    // For this primitive, we don't divide the RPM
    prim_msg_ptr->set_parameter4(dribble_primitive.getRpm());
    prim_msg_ptr->set_extra_bits(dribble_primitive.isSmallKickAllowed());
}

void ProtobufPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::SHOOT);
    prim_msg_ptr->set_parameter1(kick_primitive.getKickOrigin().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(kick_primitive.getKickOrigin().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(kick_primitive.getKickDirection().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_parameter4(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER);
    prim_msg_ptr->set_extra_bits(static_cast<uint32_t>(2 | 0));
}

void ProtobufPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::MOVE);
    prim_msg_ptr->set_parameter1(move_primitive.getDestination().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(move_primitive.getDestination().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(move_primitive.getFinalAngle().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_parameter4(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    prim_msg_ptr->set_slow(move_primitive.getMoveType() == MoveType::SLOW);

    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    extra_bits |= (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
    prim_msg_ptr->set_extra_bits(extra_bits);
}

void ProtobufPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::SPIN);
    prim_msg_ptr->set_parameter1(movespin_primitive.getDestination().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(movespin_primitive.getDestination().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(movespin_primitive.getAngularVelocity().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_parameter4(movespin_primitive.getFinalSpeed() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_extra_bits(0);
}

void ProtobufPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::PIVOT);
    prim_msg_ptr->set_parameter1(pivot_primitive.getPivotPoint().x() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter2(pivot_primitive.getPivotPoint().y() *
                                 MILLIMETERS_PER_METER);
    prim_msg_ptr->set_parameter3(pivot_primitive.getFinalAngle().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_parameter4(pivot_primitive.getPivotSpeed().toRadians() *
                                 CENTIRADIANS_PER_RADIAN);
    prim_msg_ptr->set_extra_bits(pivot_primitive.isDribblerEnabled());
}

void ProtobufPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    prim_msg_ptr = std::move(std::make_unique<RadioPrimitiveMsg>());
    prim_msg_ptr->set_prim_type(FirmwarePrimitiveType::STOP);
    prim_msg_ptr->set_extra_bits((uint32_t)stop_primitive.robotShouldCoast());
}
