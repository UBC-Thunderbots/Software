#include "software/primitive/kick_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string KickPrimitive::PRIMITIVE_NAME = "Kick Primitive";

KickPrimitive::KickPrimitive(unsigned int robot_id, const Point &kick_origin,
                             const Angle &kick_direction,
                             double kick_speed_meters_per_second)
    : robot_id(robot_id),
      kick_origin(kick_origin),
      kick_direction(kick_direction),
      kick_speed_meters_per_second(kick_speed_meters_per_second)
{
}

std::string KickPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int KickPrimitive::getRobotId() const
{
    return robot_id;
}

Point KickPrimitive::getKickOrigin() const
{
    return kick_origin;
}

Angle KickPrimitive::getKickDirection() const
{
    return kick_direction;
}

double KickPrimitive::getKickSpeed() const
{
    return kick_speed_meters_per_second;
}

void KickPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool KickPrimitive::operator==(const KickPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->kick_origin == other.kick_origin &&
           this->kick_direction == other.kick_direction &&
           this->kick_speed_meters_per_second == other.kick_speed_meters_per_second;
}

bool KickPrimitive::operator!=(const KickPrimitive &other) const
{
    return !((*this) == other);
}
