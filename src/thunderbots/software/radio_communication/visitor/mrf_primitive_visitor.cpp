#include "radio_communication/visitor/mrf_primitive_visitor.h"

RadioPrimitive MRFPrimitiveVisitor::getSerializedRadioPacket()
{
    return r_prim;
}

void MRFPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    r_prim.prim_type   = PrimitiveType::CATCH;
    r_prim.param_array = {catch_primitive.getVelocity(),
                          catch_primitive.getDribblerSpeed(),
                          catch_primitive.getMargin()};
    r_prim.extra_bits  = 0;
}

void MRFPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    r_prim.prim_type   = PrimitiveType::SHOOT;
    r_prim.param_array = {chip_primitive.getChipOrigin().x() * 1000,
                          chip_primitive.getChipOrigin().y() * 1000,
                          chip_primitive.getChipDirection().toRadians() * 100,
                          chip_primitive.getChipDistance() * 1000};
    r_prim.extra_bits  = static_cast<uint8_t>(2 | 1);
}

void MRFPrimitiveVisitor::visit(const DirectVelocityPrimitive &direct_velocity_primitive)
{
    r_prim.prim_type   = PrimitiveType::DIRECT_VELOCITY;
    r_prim.param_array = {direct_velocity_primitive.getXVelocity() * 1000,
                          direct_velocity_primitive.getYVelocity() * 1000,
                          direct_velocity_primitive.getAngularVelocity() * 100, 0};
    r_prim.extra_bits  = 0;
}

void MRFPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    r_prim.prim_type   = PrimitiveType::DIRECT_WHEELS;
    r_prim.param_array = {static_cast<double>(direct_wheels_primitive.getWheel0Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel1Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel2Power()),
                          static_cast<double>(direct_wheels_primitive.getWheel3Power())};
    r_prim.extra_bits  = 0;
}

void MRFPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    r_prim.prim_type   = PrimitiveType::SHOOT;
    r_prim.param_array = {kick_primitive.getKickOrigin().x() * 1000,
                          kick_primitive.getKickOrigin().y() * 1000,
                          kick_primitive.getKickDirection().toRadians() * 100,
                          kick_primitive.getKickSpeed() * 1000};
    r_prim.extra_bits  = static_cast<uint8_t>(2 | 0);
}

void MRFPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    r_prim.prim_type   = PrimitiveType::MOVE;
    r_prim.param_array = {move_primitive.getDestination().x() * 1000,
                          move_primitive.getDestination().y() * 1000,
                          move_primitive.getFinalAngle().toRadians() * 100,
                          move_primitive.getFinalSpeed()};

    r_prim.extra_bits = 0;
}

void MRFPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    r_prim.prim_type   = PrimitiveType::SPIN;
    r_prim.param_array = {movespin_primitive.getDestination().x() * 1000,
                          movespin_primitive.getDestination().y() * 1000,
                          movespin_primitive.getAngularVelocity().toRadians() * 100, 0};

    r_prim.extra_bits = 0;
}
void MRFPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    r_prim.prim_type   = PrimitiveType::PIVOT;
    r_prim.param_array = {pivot_primitive.getPivotPoint().x() * 1000,
                          pivot_primitive.getPivotPoint().y() * 1000,
                          pivot_primitive.getFinalAngle().toRadians() * 100,
                          pivot_primitive.getPivotRadius() * 1000};

    r_prim.extra_bits = 0;
}

void MRFPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    r_prim.prim_type   = PrimitiveType::STOP;
    r_prim.param_array = {0, 0, 0, 0};
    r_prim.extra_bits  = stop_primitive.robotShouldCoast();
}
