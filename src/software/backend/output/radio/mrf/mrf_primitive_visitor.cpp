#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"

#include "shared/constants.h"
#include "software/primitive/all_primitives.h"

// TODO: update tests for this!

RadioPrimitiveMsg CreateProtoPrimitiveVisitor::getProto()
{
    // If we've never visited a primitive (and so have never populated the
    // `radio_prim`) then throw an exception
    if (!radio_prim)
    {
        std::string err_msg = std::string(typeid(this).name()) + ": " + __func__ +
                              " called without ever having visited anything";
        throw std::runtime_error(err_msg);
    }

    return *radio_prim;
}

void CreateProtoPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_CATCH);
    radio_prim->set_parameter1(catch_primitive.getVelocity());
    radio_prim->set_parameter2(catch_primitive.getDribblerSpeed());
    radio_prim->set_parameter3(catch_primitive.getMargin());
    radio_prim->set_parameter4(0);
    radio_prim->set_extra_bits(0);
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_SHOOT);
    radio_prim->set_parameter1(chip_primitive.getChipOrigin().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(chip_primitive.getChipOrigin().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(chip_primitive.getChipDirection().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(chip_primitive.getChipDistance() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint8_t>(2 | 1));
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(RadioPrimitiveMsg::PrimitiveType::
                                  RadioPrimitiveMsg_PrimitiveType_DIRECT_VELOCITY);
    radio_prim->set_parameter1(direct_velocity_primitive.getXVelocity() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(direct_velocity_primitive.getYVelocity() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(direct_velocity_primitive.getAngularVelocity() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(0);
    radio_prim->set_extra_bits(direct_velocity_primitive.getDribblerRpm() *
                               DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(
    const DirectWheelsPrimitive &direct_wheels_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_DIRECT_WHEELS);
    radio_prim->set_parameter1(
        static_cast<float>(direct_wheels_primitive.getWheel0Power()));
    radio_prim->set_parameter2(
        static_cast<float>(direct_wheels_primitive.getWheel1Power()));
    radio_prim->set_parameter3(
        static_cast<float>(direct_wheels_primitive.getWheel2Power()));
    radio_prim->set_parameter4(
        static_cast<float>(direct_wheels_primitive.getWheel3Power()));
    radio_prim->set_extra_bits(direct_wheels_primitive.getDribblerRPM() *
                               DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_DRIBBLE);
    radio_prim->set_parameter1(dribble_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(dribble_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(dribble_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    // For this primitive, we don't divide the RPM
    radio_prim->set_parameter4(dribble_primitive.getRpm());
    radio_prim->set_extra_bits(dribble_primitive.isSmallKickAllowed());
    radio_prim->set_slow(false);
}

// TODO: make sure all comments were carried over in this file!!!!

void CreateProtoPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_SHOOT);
    radio_prim->set_parameter1(kick_primitive.getKickOrigin().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(kick_primitive.getKickOrigin().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(kick_primitive.getKickDirection().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint8_t>(2 | 0));
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_MOVE);
    radio_prim->set_parameter1(move_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(move_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(move_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    extra_bits |= (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
    radio_prim->set_extra_bits(extra_bits);
    radio_prim->set_slow(move_primitive.getMoveType() == MoveType::SLOW);
}

void CreateProtoPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_SPIN);
    radio_prim->set_parameter1(movespin_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(movespin_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(movespin_primitive.getAngularVelocity().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(movespin_primitive.getFinalSpeed() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(0);
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_PIVOT);
    radio_prim->set_parameter1(pivot_primitive.getPivotPoint().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(pivot_primitive.getPivotPoint().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(pivot_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(pivot_primitive.getPivotSpeed().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_extra_bits(pivot_primitive.isDribblerEnabled());
    radio_prim->set_slow(false);
}

void CreateProtoPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    radio_prim = RadioPrimitiveMsg();
    radio_prim->set_prim_type(
        RadioPrimitiveMsg::PrimitiveType::RadioPrimitiveMsg_PrimitiveType_STOP);
    radio_prim->set_parameter1(0);
    radio_prim->set_parameter2(0);
    radio_prim->set_parameter3(0);
    radio_prim->set_parameter4(0);
    radio_prim->set_extra_bits((uint8_t)stop_primitive.robotShouldCoast());
    radio_prim->set_slow(false);
}
