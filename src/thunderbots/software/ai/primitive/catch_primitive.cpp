#include "ai/primitive/grsim_command_primitive_visitor_catch.h"
#include "ai/primitive/visitor/primitive_visitor.h"

const std::string CatchPrimitive::PRIMITIVE_NAME = "Catch Primitive";

CatchPrimitive::CatchPrimitive(unsigned int robot_id, double velocity,
                               double dribbler_rpm, double ball_intercept_margin)
    : robot_id(robot_id),
      velocity(velocity),
      dribbler_speed(dribbler_rpm),
      margin(ball_intercept_margin)
{
}

CatchPrimitive::CatchPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id       = primitive_msg.robot_id;
    velocity       = primitive_msg.parameters.at(0);
    dribbler_speed = primitive_msg.parameters.at(1);
    margin         = primitive_msg.parameters.at(2);
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

std::vector<double> CatchPrimitive::getParameters() const
{
    std::vector<double> parameters = {velocity, dribbler_speed, margin};
    return parameters;
}

std::vector<bool> CatchPrimitive::getExtraBits() const
{
    return std::vector<bool>();
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
