#include "software/primitive/stop_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string StopPrimitive::PRIMITIVE_NAME = "Stop Primitive";

StopPrimitive::StopPrimitive(unsigned int robot_id, bool coast)
    : robot_id(robot_id), coast(coast)
{
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
