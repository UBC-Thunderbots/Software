#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"

#include "shared/constants.h"
#include "software/primitive/all_primitives.h"

RadioPrimitive MRFPrimitiveVisitor::getSerializedRadioPacket()
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

void MRFPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::CATCH;
    radio_prim->param_array = {catch_primitive.getVelocity(),
                               catch_primitive.getDribblerSpeed(),
                               catch_primitive.getMargin()};
    radio_prim->extra_bits  = 0;
}

void MRFPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::SHOOT;
    radio_prim->param_array = {
        chip_primitive.getChipOrigin().x() * MILLIMETERS_PER_METER,
        chip_primitive.getChipOrigin().y() * MILLIMETERS_PER_METER,
        chip_primitive.getChipDirection().toRadians() * CENTIRADIANS_PER_RADIAN,
        chip_primitive.getChipDistance() * MILLIMETERS_PER_METER};
    radio_prim->extra_bits = static_cast<uint8_t>(2 | 1);
}

void MRFPrimitiveVisitor::visit(const DirectVelocityPrimitive &direct_velocity_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::DIRECT_VELOCITY;
    radio_prim->param_array = {
        direct_velocity_primitive.getXVelocity() * MILLIMETERS_PER_METER,
        direct_velocity_primitive.getYVelocity() * MILLIMETERS_PER_METER,
        direct_velocity_primitive.getAngularVelocity() * CENTIRADIANS_PER_RADIAN, 0};
    radio_prim->extra_bits = direct_velocity_primitive.getDribblerRpm() *
                             DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR;
}

void MRFPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::DIRECT_WHEELS;
    radio_prim->param_array = {
        static_cast<double>(direct_wheels_primitive.getWheel0Power()),
        static_cast<double>(direct_wheels_primitive.getWheel1Power()),
        static_cast<double>(direct_wheels_primitive.getWheel2Power()),
        static_cast<double>(direct_wheels_primitive.getWheel3Power())};
    radio_prim->extra_bits = direct_wheels_primitive.getDribblerRPM() *
                             DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR;
}

void MRFPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::DRIBBLE;
    radio_prim->param_array = {
        dribble_primitive.getDestination().x() * MILLIMETERS_PER_METER,
        dribble_primitive.getDestination().y() * MILLIMETERS_PER_METER,
        dribble_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN,
        // For this primitive, we don't divide the RPM
        dribble_primitive.getRpm()};
    radio_prim->extra_bits = dribble_primitive.isSmallKickAllowed();
}

void MRFPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::SHOOT;
    radio_prim->param_array = {
        kick_primitive.getKickOrigin().x() * MILLIMETERS_PER_METER,
        kick_primitive.getKickOrigin().y() * MILLIMETERS_PER_METER,
        kick_primitive.getKickDirection().toRadians() * CENTIRADIANS_PER_RADIAN,
        kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER};
    radio_prim->extra_bits = static_cast<uint8_t>(2 | 0);
}

void MRFPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::MOVE;
    radio_prim->param_array = {
        move_primitive.getDestination().x() * MILLIMETERS_PER_METER,
        move_primitive.getDestination().y() * MILLIMETERS_PER_METER,
        move_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN,
        move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER};
    radio_prim->slow       = move_primitive.getMoveType() == MoveType::SLOW;
    radio_prim->extra_bits = 0;
    radio_prim->extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    radio_prim->extra_bits |=
        (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    radio_prim->extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
}

void MRFPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::SPIN;
    radio_prim->param_array = {
        movespin_primitive.getDestination().x() * MILLIMETERS_PER_METER,
        movespin_primitive.getDestination().y() * MILLIMETERS_PER_METER,
        movespin_primitive.getAngularVelocity().toRadians() * CENTIRADIANS_PER_RADIAN,
        movespin_primitive.getFinalSpeed() * MILLIMETERS_PER_METER};
    radio_prim->extra_bits = 0;
}

void MRFPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::PIVOT;
    radio_prim->param_array = {
        pivot_primitive.getPivotPoint().x() * MILLIMETERS_PER_METER,
        pivot_primitive.getPivotPoint().y() * MILLIMETERS_PER_METER,
        pivot_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN,
        pivot_primitive.getPivotSpeed().toRadians() * CENTIRADIANS_PER_RADIAN};
    radio_prim->extra_bits = pivot_primitive.isDribblerEnabled();
}

void MRFPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->prim_type   = FirmwarePrimitiveType::STOP;
    radio_prim->param_array = {0, 0, 0, 0};
    radio_prim->extra_bits  = (uint8_t)stop_primitive.robotShouldCoast();
}
