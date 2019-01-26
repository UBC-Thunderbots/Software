#include "ai/primitive/kick_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

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

KickPrimitive::KickPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id                     = primitive_msg.robot_id;
    double kick_origin_x         = primitive_msg.parameters.at(0);
    double kick_origin_y         = primitive_msg.parameters.at(1);
    kick_origin                  = Point(kick_origin_x, kick_origin_y);
    kick_direction               = Angle::ofRadians(primitive_msg.parameters.at(2));
    kick_speed_meters_per_second = primitive_msg.parameters.at(3);
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
