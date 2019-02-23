#include "ai/primitive/grsim_command_primitive_visitor_movespin.h"
#include "ai/primitive/visitor/primitive_visitor.h"

const std::string MoveSpinPrimitive::PRIMITIVE_NAME = "MoveSpin Primitive";

MoveSpinPrimitive::MoveSpinPrimitive(unsigned int robot_id, const Point &dest,
                                     const AngularVelocity &angular_vel)
    : robot_id(robot_id), dest(dest), angular_vel(angular_vel)
{
}

MoveSpinPrimitive::MoveSpinPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id      = primitive_msg.robot_id;
    double dest_x = primitive_msg.parameters.at(0);
    double dest_y = primitive_msg.parameters.at(1);
    dest          = Point(dest_x, dest_y);
    angular_vel   = Angle::ofRadians(primitive_msg.parameters.at(2));
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

std::vector<double> MoveSpinPrimitive::getParameters() const
{
    std::vector<double> parameters = {dest.x(), dest.y(), angular_vel.toRadians()};

    return parameters;
}

std::vector<bool> MoveSpinPrimitive::getExtraBits() const
{
    return std::vector<bool>();
}

void MoveSpinPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MoveSpinPrimitive::operator==(const MoveSpinPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->angular_vel == other.angular_vel;
}

bool MoveSpinPrimitive::operator!=(const MoveSpinPrimitive &other) const
{
    return !((*this) == other);
}
