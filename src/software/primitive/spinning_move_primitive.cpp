#include "software/primitive/spinning_move_primitive.h"

const std::string SpinningMovePrimitive::PRIMITIVE_NAME = "SpinningMove Primitive";

SpinningMovePrimitive::SpinningMovePrimitive(unsigned int robot_id, const Point &dest,
                                             const AngularVelocity &angular_vel,
                                             double final_speed)
    : robot_id(robot_id), dest(dest), angular_vel(angular_vel), final_speed(final_speed)
{
}

std::string SpinningMovePrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int SpinningMovePrimitive::getRobotId() const
{
    return robot_id;
}

Point SpinningMovePrimitive::getDestination() const
{
    return dest;
}

AngularVelocity SpinningMovePrimitive::getAngularVelocity() const
{
    return angular_vel;
}

double SpinningMovePrimitive::getFinalSpeed() const
{
    return final_speed;
}

void SpinningMovePrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool SpinningMovePrimitive::operator==(const SpinningMovePrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->angular_vel == other.angular_vel &&
           this->final_speed == other.final_speed;
}

bool SpinningMovePrimitive::operator!=(const SpinningMovePrimitive &other) const
{
    return !((*this) == other);
}
