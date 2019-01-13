#include "ai/primitive/catch_primitive.h"

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

PrimitiveType CatchPrimitive::getPrimitiveType() const
{
    return PrimitiveType::CATCH;
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
