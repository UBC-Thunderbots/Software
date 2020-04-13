#include "software/primitive/movespin_primitive.h"

#include "software/primitive/primitive_visitor.h"

const std::string MoveSpinPrimitive::PRIMITIVE_NAME = "MoveSpin Primitive";

MoveSpinPrimitive::MoveSpinPrimitive(unsigned int robot_id, const Point &dest,
                                     const AngularVelocity &angular_vel,
                                     double final_speed)
    : robot_id(robot_id), dest(dest), angular_vel(angular_vel), final_speed(final_speed)
{
}

std::string MoveSpinPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int MoveSpinPrimitive::getRobotId() const
{
    return robot_id;
}

Point MoveSpinPrimitive::getDestination() const
{
    return dest;
}

AngularVelocity MoveSpinPrimitive::getAngularVelocity() const
{
    return angular_vel;
}

double MoveSpinPrimitive::getFinalSpeed() const
{
    return final_speed;
}

void MoveSpinPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MoveSpinPrimitive::operator==(const MoveSpinPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->angular_vel == other.angular_vel &&
           this->final_speed == other.final_speed;
}

bool MoveSpinPrimitive::operator!=(const MoveSpinPrimitive &other) const
{
    return !((*this) == other);
}
