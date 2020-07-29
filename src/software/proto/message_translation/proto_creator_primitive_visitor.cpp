#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

#include "shared/constants.h"
#include "software/primitive/all_primitives.h"

PrimitiveMsg ProtoCreatorPrimitiveVisitor::createPrimitiveMsg(const Primitive &primitive)
{
    primitive.accept(*static_cast<PrimitiveVisitor *>(this));
    return *prim;
}

void ProtoCreatorPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(
        static_cast<float>(chip_primitive.getChipOrigin().x() * MILLIMETERS_PER_METER));
    params->set_parameter2(
        static_cast<float>(chip_primitive.getChipOrigin().y() * MILLIMETERS_PER_METER));
    params->set_parameter3(static_cast<float>(
        chip_primitive.getChipDirection().toRadians() * CENTIRADIANS_PER_RADIAN));
    params->set_parameter4(
        static_cast<float>(chip_primitive.getChipDistance() * MILLIMETERS_PER_METER));
    params->set_extra_bits(static_cast<uint8_t>(2 | 1));
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_shoot(params);
}

void ProtoCreatorPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(static_cast<float>(direct_velocity_primitive.getXVelocity() *
                                              MILLIMETERS_PER_METER));
    params->set_parameter2(static_cast<float>(direct_velocity_primitive.getYVelocity() *
                                              MILLIMETERS_PER_METER));
    params->set_parameter3(static_cast<float>(
        direct_velocity_primitive.getAngularVelocity() * CENTIRADIANS_PER_RADIAN));
    params->set_parameter4(static_cast<float>(0));
    params->set_extra_bits(
        static_cast<uint8_t>(direct_velocity_primitive.getDribblerRpm() *
                             DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR));
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_direct_velocity(params);
}

void ProtoCreatorPrimitiveVisitor::visit(
    const DirectWheelsPrimitive &direct_wheels_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(static_cast<float>(direct_wheels_primitive.getWheel0Power()));
    params->set_parameter2(static_cast<float>(direct_wheels_primitive.getWheel1Power()));
    params->set_parameter3(static_cast<float>(direct_wheels_primitive.getWheel2Power()));
    params->set_parameter4(static_cast<float>(direct_wheels_primitive.getWheel3Power()));
    params->set_extra_bits(static_cast<uint8_t>(direct_wheels_primitive.getDribblerRPM() *
                                                DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR));
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_direct_wheels(params);
}

void ProtoCreatorPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(
        static_cast<float>(kick_primitive.getKickOrigin().x() * MILLIMETERS_PER_METER));
    params->set_parameter2(
        static_cast<float>(kick_primitive.getKickOrigin().y() * MILLIMETERS_PER_METER));
    params->set_parameter3(static_cast<float>(
        kick_primitive.getKickDirection().toRadians() * CENTIRADIANS_PER_RADIAN));
    params->set_parameter4(
        static_cast<float>(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER));
    params->set_extra_bits(static_cast<uint8_t>(2 | 0));
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_shoot(params);
}

void ProtoCreatorPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(
        static_cast<float>(move_primitive.getDestination().x() * MILLIMETERS_PER_METER));
    params->set_parameter2(
        static_cast<float>(move_primitive.getDestination().y() * MILLIMETERS_PER_METER));
    params->set_parameter3(static_cast<float>(move_primitive.getFinalAngle().toRadians() *
                                              CENTIRADIANS_PER_RADIAN));
    params->set_parameter4(
        static_cast<float>(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER));
    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutochickType() == AutochickType::AUTOKICK) * 0x01;
    extra_bits |= (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutochickType() == AutochickType::AUTOCHIP) * 0x04;
    params->set_extra_bits(extra_bits);
    params->set_slow(move_primitive.getMoveType() == MoveType::SLOW);

    prim = PrimitiveMsg();
    prim->set_allocated_move(params);
}

void ProtoCreatorPrimitiveVisitor::visit(
    const SpinningMovePrimitive &spinning_move_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(static_cast<float>(
        spinning_move_primitive.getDestination().x() * MILLIMETERS_PER_METER));
    params->set_parameter2(static_cast<float>(
        spinning_move_primitive.getDestination().y() * MILLIMETERS_PER_METER));
    params->set_parameter3(
        static_cast<float>(spinning_move_primitive.getAngularVelocity().toRadians() *
                           CENTIRADIANS_PER_RADIAN));
    params->set_parameter4(static_cast<float>(spinning_move_primitive.getFinalSpeed() *
                                              MILLIMETERS_PER_METER));
    params->set_extra_bits(0);
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_spinning_move(params);
}

void ProtoCreatorPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    PrimitiveParamsMsg *params = new PrimitiveParamsMsg();
    params->set_parameter1(static_cast<float>(0));
    params->set_parameter2(static_cast<float>(0));
    params->set_parameter3(static_cast<float>(0));
    params->set_parameter4(static_cast<float>(0));
    params->set_extra_bits((uint8_t)stop_primitive.robotShouldCoast());
    params->set_slow(false);

    prim = PrimitiveMsg();
    prim->set_allocated_stop(params);
}
