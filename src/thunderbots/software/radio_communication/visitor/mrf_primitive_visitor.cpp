#include "radio_communication/visitor/mrf_primitive_visitor.h"

#include "shared/constants.h"

std::optional<RadioPrimitive> MRFPrimitiveVisitor::getSerializedRadioPacket()
{
    return r_prim;
}

void MRFPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::CATCH;
    r_prim.param_array = {catch_primitive.getVelocity(),
                          catch_primitive.getDribblerSpeed(),
                          catch_primitive.getMargin()};
    r_prim.extra_bits  = 0;
    prim               = r_prim;
}

void MRFPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::SHOOT;
    r_prim.param_array = {
        chip_primitive.getChipOrigin().x() * MILLIMETERS_PER_METER,
        chip_primitive.getChipOrigin().y() * MILLIMETERS_PER_METER,
        chip_primitive.getChipDirection().toRadians() * CENTIRADIANS_PER_RADIAN,
        chip_primitive.getChipDistance() * MILLIMETERS_PER_METER};
    r_prim.extra_bits = static_cast<uint8_t>(2 | 1);
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const DirectVelocityPrimitive &direct_velocity_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::DIRECT_VELOCITY;
    r_prim.param_array = {
        direct_velocity_primitive.getXVelocity() * MILLIMETERS_PER_METER,
        direct_velocity_primitive.getYVelocity() * MILLIMETERS_PER_METER,
        direct_velocity_primitive.getAngularVelocity() * CENTIRADIANS_PER_RADIAN, 0};
    r_prim.extra_bits = 0;
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::DIRECT_WHEELS;
    r_prim.param_array = {static_cast<double>(direct_wheels_primitive.getWheel0Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel1Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel2Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel3Power())};
    r_prim.extra_bits  = 0;
    prim               = r_prim;
}

void MRFPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::SHOOT;
    r_prim.param_array = {
        kick_primitive.getKickOrigin().x() * MILLIMETERS_PER_METER,
        kick_primitive.getKickOrigin().y() * MILLIMETERS_PER_METER,
        kick_primitive.getKickDirection().toRadians() * CENTIRADIANS_PER_RADIAN,
        kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER};
    r_prim.extra_bits = static_cast<uint8_t>(2 | 0);
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::MOVE;
    r_prim.param_array = {
        move_primitive.getDestination().x() * MILLIMETERS_PER_METER,
        move_primitive.getDestination().y() * MILLIMETERS_PER_METER,
        move_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN,
        move_primitive.getFinalSpeed()};
    r_prim.extra_bits = 0;
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::SPIN;
    r_prim.param_array = {
        movespin_primitive.getDestination().x() * MILLIMETERS_PER_METER,
        movespin_primitive.getDestination().y() * MILLIMETERS_PER_METER,
        movespin_primitive.getAngularVelocity().toRadians() * CENTIRADIANS_PER_RADIAN, 0};
    r_prim.extra_bits = 0;
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::PIVOT;
    r_prim.param_array = {
        pivot_primitive.getPivotPoint().x() * MILLIMETERS_PER_METER,
        pivot_primitive.getPivotPoint().y() * MILLIMETERS_PER_METER,
        pivot_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN,
        pivot_primitive.getPivotRadius() * MILLIMETERS_PER_METER};
    r_prim.extra_bits = 0;
    prim              = r_prim;
}

void MRFPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    r_prim.prim_type   = FirmwarePrimitiveType::STOP;
    r_prim.param_array = {0, 0, 0, 0};
    r_prim.extra_bits  = stop_primitive.robotShouldCoast();
    prim               = r_prim;
}
