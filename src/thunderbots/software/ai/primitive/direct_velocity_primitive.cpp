#include "ai/primitive/direct_velocity_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string DirectVelocityPrimitive::PRIMITIVE_NAME = "Direct Velocity Primitive";

DirectVelocityPrimitive::DirectVelocityPrimitive(unsigned int robot_id, double x_velocity,
                                                 double y_velocity,
                                                 double angular_velocity,
                                                 double dribbler_rpm)
    : robot_id(robot_id),
      x_velocity(x_velocity),
      y_velocity(y_velocity),
      angular_velocity(angular_velocity),
      dribbler_rpm(dribbler_rpm)
{
}

DirectVelocityPrimitive::DirectVelocityPrimitive(
    const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id         = primitive_msg.robot_id;
    x_velocity       = primitive_msg.parameters.at(0);
    y_velocity       = primitive_msg.parameters.at(1);
    angular_velocity = primitive_msg.parameters.at(2);
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
    return x_velocity;
}

double DirectVelocityPrimitive::getYVelocity() const
{
    return y_velocity;
}

double DirectVelocityPrimitive::getAngularVelocity() const
{
    return angular_velocity;
}

double DirectVelocityPrimitive::getDribblerRpm() const
{
    return dribbler_rpm;
}

std::vector<double> DirectVelocityPrimitive::getParameters() const
{
    std::vector<double> parameters = {x_velocity, y_velocity, angular_velocity,
                                      dribbler_rpm};

    return parameters;
}

std::vector<bool> DirectVelocityPrimitive::getExtraBits() const
{
    return std::vector<bool>();
}

void DirectVelocityPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DirectVelocityPrimitive::operator==(const DirectVelocityPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->x_velocity == other.x_velocity &&
           this->y_velocity == other.y_velocity &&
           this->angular_velocity == other.angular_velocity &&
           this->dribbler_rpm == other.dribbler_rpm;
}

bool DirectVelocityPrimitive::operator!=(const DirectVelocityPrimitive &other) const
{
    return !((*this) == other);
}
