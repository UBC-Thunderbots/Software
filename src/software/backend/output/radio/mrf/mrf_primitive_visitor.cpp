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
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::CATCH);
    radio_prim->add_parameter(catch_primitive.getVelocity());
    radio_prim->add_parameter(catch_primitive.getDribblerSpeed());
    radio_prim->add_parameter(catch_primitive.getMargin());
    radio_prim->set_extra_bits(0);
}

void MRFPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SHOOT);
    radio_prim->add_parameter(chip_primitive.getChipOrigin().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(chip_primitive.getChipOrigin().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(chip_primitive.getChipDirection().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->add_parameter(chip_primitive.getChipDistance() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint32_t>(2 | 1));
}

void MRFPrimitiveVisitor::visit(const DirectVelocityPrimitive &direct_velocity_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DIRECT_VELOCITY);
    radio_prim->add_parameter(direct_velocity_primitive.getXVelocity() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(direct_velocity_primitive.getYVelocity() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(direct_velocity_primitive.getAngularVelocity() * CENTIRADIANS_PER_RADIAN);
    radio_prim->set_extra_bits(direct_velocity_primitive.getDribblerRpm() *
                             DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void MRFPrimitiveVisitor::visit(const DirectWheelsPrimitive &direct_wheels_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DIRECT_WHEELS);
    radio_prim->add_parameter(static_cast<double>(direct_wheels_primitive.getWheel0Power()));
    radio_prim->add_parameter(static_cast<double>(direct_wheels_primitive.getWheel1Power()));
    radio_prim->add_parameter(static_cast<double>(direct_wheels_primitive.getWheel2Power()));
    radio_prim->add_parameter(static_cast<double>(direct_wheels_primitive.getWheel3Power()));
    radio_prim->set_extra_bits(direct_wheels_primitive.getDribblerRPM() *
                             DRIBBLER_RPM_TO_RADIO_CONVERSION_FACTOR);
}

void MRFPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::DRIBBLE);
    radio_prim->add_parameter(dribble_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(dribble_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(dribble_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN);
    // For this primitive, we don't divide the RPM
    radio_prim->add_parameter(dribble_primitive.getRpm());
    radio_prim->set_extra_bits(dribble_primitive.isSmallKickAllowed());
}

void MRFPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SHOOT);
    radio_prim->add_parameter(kick_primitive.getKickOrigin().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(kick_primitive.getKickOrigin().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(kick_primitive.getKickDirection().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->add_parameter(kick_primitive.getKickSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(static_cast<uint32_t>(2 | 0));
}

void MRFPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::MOVE);
    radio_prim->add_parameter(move_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(move_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(move_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->add_parameter(move_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_slow(move_primitive.getMoveType() == MoveType::SLOW);

    uint32_t extra_bits = 0;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOKICK) * 0x01;
    extra_bits |=
        (move_primitive.getDribblerEnable() == DribblerEnable::ON) * 0x02;
    extra_bits |= (move_primitive.getAutoKickType() == AUTOCHIP) * 0x04;
    radio_prim->set_extra_bits(extra_bits);
}

void MRFPrimitiveVisitor::visit(const MoveSpinPrimitive &movespin_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::SPIN);
    radio_prim->add_parameter(movespin_primitive.getDestination().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(movespin_primitive.getDestination().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(movespin_primitive.getAngularVelocity().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->add_parameter(movespin_primitive.getFinalSpeed() * MILLIMETERS_PER_METER);
    radio_prim->set_extra_bits(0);
}

void MRFPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::PIVOT);
    radio_prim->add_parameter(pivot_primitive.getPivotPoint().x() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(pivot_primitive.getPivotPoint().y() * MILLIMETERS_PER_METER);
    radio_prim->add_parameter(pivot_primitive.getFinalAngle().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->add_parameter(pivot_primitive.getPivotSpeed().toRadians() * CENTIRADIANS_PER_RADIAN);
    radio_prim->set_extra_bits(pivot_primitive.isDribblerEnabled());
}

void MRFPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    radio_prim              = RadioPrimitive();
    radio_prim->set_prim_type(FirmwarePrimitiveType::STOP);
    radio_prim->set_extra_bits((uint32_t)stop_primitive.robotShouldCoast());
}
