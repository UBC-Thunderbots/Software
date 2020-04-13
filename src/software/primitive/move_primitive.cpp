#include "software/primitive/move_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string MovePrimitive::PRIMITIVE_NAME = "Move Primitive";

MovePrimitive::MovePrimitive(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double final_speed,
                             DribblerEnable enable_dribbler, MoveType move_type,
                             AutokickType autokick)
    : robot_id(robot_id),
      dest(dest),
      final_angle(final_angle),
      final_speed(final_speed),
      enable_dribbler(enable_dribbler),
      move_type(move_type),
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

DribblerEnable MovePrimitive::getDribblerEnable() const
{
    return enable_dribbler;
}

MoveType MovePrimitive::getMoveType() const
{
    return move_type;
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
           this->enable_dribbler == other.enable_dribbler &&
           this->move_type == other.move_type && this->autokick == other.autokick;
}

bool MovePrimitive::operator!=(const MovePrimitive &other) const
{
    return !((*this) == other);
}
