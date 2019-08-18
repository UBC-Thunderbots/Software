#include "ai/primitive/move_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string MovePrimitive::PRIMITIVE_NAME = "Move Primitive";

MovePrimitive::MovePrimitive(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double final_speed,
                             bool enable_dribbler, bool slow, AutokickType autokick)
    : robot_id(robot_id),
      dest(dest),
      final_angle(final_angle),
      final_speed(final_speed),
      enable_dribbler(enable_dribbler),
      slow(slow),
      autokick(autokick)
{
}

std::string MovePrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int MovePrimitive::getRobotId() const
{
    return robot_id;
}

Point MovePrimitive::getDestination() const
{
    return dest;
}

Angle MovePrimitive::getFinalAngle() const
{
    return final_angle;
}

double MovePrimitive::getFinalSpeed() const
{
    return final_speed;
}

AutokickType MovePrimitive::getAutoKickType() const
{
    return autokick;
}

bool MovePrimitive::isDribblerEnabled() const
{
    return enable_dribbler;
}

bool MovePrimitive::isSlowEnabled() const
{
    return slow;
}

void MovePrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MovePrimitive::operator==(const MovePrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->final_angle == other.final_angle &&
           this->final_speed == other.final_speed &&
           this->enable_dribbler == other.enable_dribbler && this->slow == other.slow &&
           this->autokick == other.autokick;
}

bool MovePrimitive::operator!=(const MovePrimitive &other) const
{
    return !((*this) == other);
}
