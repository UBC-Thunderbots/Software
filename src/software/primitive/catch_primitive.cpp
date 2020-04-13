#include "software/primitive/catch_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string CatchPrimitive::PRIMITIVE_NAME = "Catch Primitive";

CatchPrimitive::CatchPrimitive(unsigned int robot_id, double velocity,
                               double dribbler_rpm, double ball_intercept_margin)
    : robot_id(robot_id),
      velocity(velocity),
      dribbler_speed(dribbler_rpm),
      margin(ball_intercept_margin)
{
}

std::string CatchPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int CatchPrimitive::getRobotId() const
{
    return robot_id;
}

double CatchPrimitive::getVelocity() const
{
    return velocity;
}

double CatchPrimitive::getDribblerSpeed() const
{
    return dribbler_speed;
}

double CatchPrimitive::getMargin() const
{
    return margin;
}

void CatchPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool CatchPrimitive::operator==(const CatchPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->velocity == other.velocity &&
           this->dribbler_speed == other.dribbler_speed && this->margin == other.margin;
}

bool CatchPrimitive::operator!=(const CatchPrimitive &other) const
{
    return !((*this) == other);
}
