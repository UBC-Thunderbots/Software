#include "software/primitive/dribble_primitive.h"

const std::string DribblePrimitive::PRIMITIVE_NAME = "Dribble Primitive";

DribblePrimitive::DribblePrimitive(unsigned int robot_id, const Point &dest,
                                   const Angle &final_angle, double rpm,
                                   bool small_kick_allowed)
    : robot_id(robot_id),
      dest(dest),
      final_angle(final_angle),
      rpm(rpm),
      small_kick_allowed(small_kick_allowed)

{
}

std::string DribblePrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int DribblePrimitive::getRobotId() const
{
    return robot_id;
}

Point DribblePrimitive::getDestination() const
{
    return dest;
}

Angle DribblePrimitive::getFinalAngle() const
{
    return final_angle;
}

double DribblePrimitive::getRpm() const
{
    return rpm;
}

bool DribblePrimitive::isSmallKickAllowed() const
{
    return small_kick_allowed;
}

void DribblePrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DribblePrimitive::operator==(const DribblePrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->final_angle == other.final_angle && this->rpm == other.rpm &&
           this->small_kick_allowed == other.small_kick_allowed;
}

bool DribblePrimitive::operator!=(const DribblePrimitive &other) const
{
    return !((*this) == other);
}
