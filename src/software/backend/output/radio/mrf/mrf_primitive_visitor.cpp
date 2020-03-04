#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"

#include "shared/constants.h"
#include "shared/proto/primitive.pb.h"
#include "software/ai/primitive/all_primitives.h"

std::string MRFPrimitiveVisitor::getSerializedRadioPacket()
{
    // If we've never visited a primitive (and so have never populated the
    // `radio_prim`) then throw an exception
    if (!radio_prim)
    {
        std::string err_msg = std::string(typeid(this).name()) + ": " + __func__ +
                              " called without ever having visited anything";
        throw std::runtime_error(err_msg);
    }

    std::string data;
    radio_prim->SerializeToString(&data);
    return data;
}

void MRFPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::CATCH);
    radio_prim->set_parameter1(catch_primitive.getVelocity());
    radio_prim->set_parameter2(catch_primitive.getDribblerSpeed());
    radio_prim->set_parameter3(catch_primitive.getMargin());
    radio_prim->set_extra_bits(0);

    // unused
    radio_prim->set_parameter4(-1);
}

void MRFPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SHOOT);
    radio_prim->set_parameter1(chip_primitive.getChipOrigin().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(chip_primitive.getChipOrigin().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(chip_primitive.getChipDirection().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(chip_primitive.getChipDistance() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint32_t>(2 | 1));
}

void MRFPrimitiveVisitor::visit(const DirectVelocityPrimitive &direct_velocity_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DIRECT_VELOCITY);
    radio_prim->set_parameter1(direct_velocity_primitive.getXVelocity() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(direct_velocity_primitive.getYVelocity() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(direct_velocity_primitive.getAngularVelocity() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_extra_bits(direct_velocity_primitive.getDribblerRpm() *
                               DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void MRFPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DIRECT_WHEELS);
    radio_prim->set_parameter1(
        static_cast<double>(direct_wheels_primitive.getWheel0Power()));
    radio_prim->set_parameter2(
        static_cast<double>(direct_wheels_primitive.getWheel1Power()));
    radio_prim->set_parameter3(
        static_cast<double>(direct_wheels_primitive.getWheel2Power()));
    radio_prim->set_parameter4(
        static_cast<double>(direct_wheels_primitive.getWheel3Power()));
    radio_prim->set_extra_bits(direct_wheels_primitive.getDribblerRPM() *
                               DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void MRFPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DRIBBLE);
    radio_prim->set_parameter1(dribble_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(dribble_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(dribble_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    // For this primitive, we don't divide the RPM
    radio_prim->set_parameter4(dribble_primitive.getRpm());
    radio_prim->set_extra_bits(dribble_primitive.isSmallKickAllowed());

    // unused
    radio_prim->set_parameter4(-1);
}

void MRFPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SHOOT);
    radio_prim->set_parameter1(kick_primitive.getKickOrigin().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(kick_primitive.getKickOrigin().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(kick_primitive.getKickDirection().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint32_t>(2 | 0));
}

void MRFPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::MOVE);
    radio_prim->set_parameter1(move_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(move_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(move_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_slow(move_primitive.getMoveType() == MoveType::SLOW);

    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    extra_bits |= (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
    radio_prim->set_extra_bits(extra_bits);
}

void MRFPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SPIN);
    radio_prim->set_parameter1(movespin_primitive.getDestination().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(movespin_primitive.getDestination().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(movespin_primitive.getAngularVelocity().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(movespin_primitive.getFinalSpeed() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(0);
}

void MRFPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::PIVOT);
    radio_prim->set_parameter1(pivot_primitive.getPivotPoint().x() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter2(pivot_primitive.getPivotPoint().y() *
                               MILLIMETERS_PER_METER);
    radio_prim->set_parameter3(pivot_primitive.getFinalAngle().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_parameter4(pivot_primitive.getPivotSpeed().toRadians() *
                               CENTIRADIANS_PER_RADIAN);
    radio_prim->set_extra_bits(pivot_primitive.isDribblerEnabled());
}

void MRFPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    radio_prim = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::STOP);
    radio_prim->set_extra_bits((uint32_t)stop_primitive.robotShouldCoast());

    // unused
    radio_prim->set_parameter1(-1);
    radio_prim->set_parameter2(-1);
    radio_prim->set_parameter3(-1);
    radio_prim->set_parameter4(-1);
}
