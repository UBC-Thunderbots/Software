#include "ai/primitive/catch_primative.h"

const std::string CatchPrimitive::PRIMITIVE_NAME = "Catch Primitive";

CatchPrimitive::CatchPrimitive(unsigned int robot_id, double velocity,
                               double dribbler_speed, double margin)
    : robot_id(robot_id),
      velocity(velocity),
      dribbler_speed(dribbler_speed),
      margin(margin)
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

std::vector<double> CatchPrimitive::getParameterArray() const
{
    std::vector<double> parameters = {velocity, dribbler_speed, margin};
    return parameters;
}

std::vector<bool> CatchPrimitive::getExtraBitArray() const
{
    return std::vector<bool>();
}
