#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

#include "shared/constants.h"
#include "software/primitive/all_primitives.h"

PrimitiveMsg ProtoCreatorPrimitiveVisitor::getProto()
{
    // If we've never visited a primitive (and so have never populated the
    // `prim`) then throw an exception
    if (!prim)
    {
        std::string err_msg = std::string(typeid(this).name()) + ": " + __func__ +
                              " called without ever having visited anything";
        throw std::runtime_error(err_msg);
    }

    return *prim;
}

PrimitiveMsg ProtoCreatorPrimitiveVisitor::createPrimitiveMsg(const Primitive &primitive)
{
    primitive.accept(*static_cast<PrimitiveVisitor *>(this));
    return getProto();
}

void ProtoCreatorPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_CATCH);
    prim->set_parameter1(catch_primitive.getVelocity());
    prim->set_parameter2(catch_primitive.getDribblerSpeed());
    prim->set_parameter3(catch_primitive.getMargin());
    prim->set_parameter4(0);
    prim->set_extra_bits(0);
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_SHOOT);
    prim->set_parameter1(chip_primitive.getChipOrigin().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(chip_primitive.getChipOrigin().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(chip_primitive.getChipDirection().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(chip_primitive.getChipDistance() * MILLIMETERS_PER_METER);
    prim->set_extra_bits(static_cast<uint8_t>(2 | 1));
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(
        PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_DIRECT_VELOCITY);
    prim->set_parameter1(direct_velocity_primitive.getXVelocity() *
                         MILLIMETERS_PER_METER);
    prim->set_parameter2(direct_velocity_primitive.getYVelocity() *
                         MILLIMETERS_PER_METER);
    prim->set_parameter3(direct_velocity_primitive.getAngularVelocity() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(0);
    prim->set_extra_bits(direct_velocity_primitive.getDribblerRpm() *
                         DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(
    const DirectWheelsPrimitive &direct_wheels_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(
        PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_DIRECT_WHEELS);
    prim->set_parameter1(static_cast<float>(direct_wheels_primitive.getWheel0Power()));
    prim->set_parameter2(static_cast<float>(direct_wheels_primitive.getWheel1Power()));
    prim->set_parameter3(static_cast<float>(direct_wheels_primitive.getWheel2Power()));
    prim->set_parameter4(static_cast<float>(direct_wheels_primitive.getWheel3Power()));
    prim->set_extra_bits(direct_wheels_primitive.getDribblerRPM() *
                         DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_DRIBBLE);
    prim->set_parameter1(dribble_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(dribble_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(dribble_primitive.getFinalAngle().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    // For this primitive, we don't divide the RPM
    prim->set_parameter4(dribble_primitive.getRpm());
    prim->set_extra_bits(dribble_primitive.isSmallKickAllowed());
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_SHOOT);
    prim->set_parameter1(kick_primitive.getKickOrigin().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(kick_primitive.getKickOrigin().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(kick_primitive.getKickDirection().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER);
    prim->set_extra_bits(static_cast<uint8_t>(2 | 0));
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_MOVE);
    prim->set_parameter1(move_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(move_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(move_primitive.getFinalAngle().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    extra_bits |= (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
    prim->set_extra_bits(extra_bits);
    prim->set_slow(move_primitive.getMoveType() == MoveType::SLOW);
}

void ProtoCreatorPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_SPIN);
    prim->set_parameter1(movespin_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(movespin_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(movespin_primitive.getAngularVelocity().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(movespin_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    prim->set_extra_bits(0);
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_PIVOT);
    prim->set_parameter1(pivot_primitive.getPivotPoint().x() * MILLIMETERS_PER_METER);
    prim->set_parameter2(pivot_primitive.getPivotPoint().y() * MILLIMETERS_PER_METER);
    prim->set_parameter3(pivot_primitive.getFinalAngle().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_parameter4(pivot_primitive.getPivotSpeed().toRadians() *
                         CENTIRADIANS_PER_RADIAN);
    prim->set_extra_bits(pivot_primitive.isDribblerEnabled());
    prim->set_slow(false);
}

void ProtoCreatorPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    prim = PrimitiveMsg();
    prim->set_prim_type(PrimitiveMsg::PrimitiveType::PrimitiveMsg_PrimitiveType_STOP);
    prim->set_parameter1(0);
    prim->set_parameter2(0);
    prim->set_parameter3(0);
    prim->set_parameter4(0);
    prim->set_extra_bits((uint8_t)stop_primitive.robotShouldCoast());
    prim->set_slow(false);
}
