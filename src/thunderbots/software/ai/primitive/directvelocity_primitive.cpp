#include "ai/primitive/directvelocity_primitive.h"

const std::string DirectVelocityPrimitive::PRIMITIVE_NAME = "DirectVelocity Primitive";

DirectVelocityPrimitive::DirectVelocityPrimitive(unsigned int robot_id, double x_vel, double y_vel,
                                                 AngularVelocity angular_vel, double dribbler_rpm)
        : robot_id(robot_id), x_vel(x_vel), y_vel(y_vel), angular_vel(angular_vel), dribbler_rpm(dribbler_rpm)
{
}

DirectVelocityPrimitive::DirectVelocityPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id         = primitive_msg.robot_id;
    double x_vel     = primitive_msg.parameters.at(0);
    double y_vel     = primitive_msg.parameters.at(1);
    angular_vel      = Angle::ofRadians(primitive_msg.parameters.at(2));
    dribbler_rpm     = primitive_msg.parameters.at(3);
}

std::string DirectVelocityPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int DirectVelocityPrimitive::getRobotId() const
{
    return robot_id;
}

double DirectVelocityPrimitive::getXVelocity() const
{
    return x_vel;
}

double DirectVelocityPrimitive::getYVelocity() const
{
    return y_vel;
}

AngularVelocity DirectVelocityPrimitive::getAngularVelocity() const
{
    return angular_vel;
}

double DirectVelocityPrimitive::getDribblerRPM() const
{
    return dribbler_rpm;
}


std::vector<double> DirectVelocityPrimitive::getParameterArray() const
{
    std::vector<double> parameters = {x_vel, y_vel, angular_vel.toRadians(),
                                      dribbler_rpm};

    return parameters;
}

std::vector<bool> DirectVelocityPrimitive::getExtraBitArray() const
{
    return std::vector<bool>();
}