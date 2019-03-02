#include "ai/primitive/stop_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string StopPrimitive::PRIMITIVE_NAME = "Stop Primitive";

StopPrimitive::StopPrimitive(unsigned int robot_id, bool coast)
    : robot_id(robot_id), coast(coast)
{
}

StopPrimitive::StopPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id = primitive_msg.robot_id;
    coast    = primitive_msg.extra_bits.at(0);
}

std::string StopPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int StopPrimitive::getRobotId() const
{
    return robot_id;
}


bool StopPrimitive::robotShouldCoast() const
{
    return coast;
}

std::vector<double> StopPrimitive::getParameters() const
{
    return std::vector<double>();
}

std::vector<bool> StopPrimitive::getExtraBits() const
{
    std::vector<bool> parameter = {coast};

    return parameter;
}

void StopPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool StopPrimitive::operator==(const StopPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->coast == other.coast;
}

bool StopPrimitive::operator!=(const StopPrimitive &other) const
{
    return !((*this) == other);
}
