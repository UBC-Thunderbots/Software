#include "ai/primitive/dribble_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string DribblePrimitive::PRIMITIVE_NAME = "Dribble Primitive";

DribblePrimitive::DribblePrimitive(unsigned int robot_id, const Point &dest,
                                   const Angle &final_angle, double rpm,
                                   bool small_kick_allowed)
    : robot_id(robot_id),
      dest(dest),
      final_angle(final_angle),
      rpm(rpm),
      small_kick_allowed(small_kick_allowed)

{
}
DribblePrimitive::DribblePrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id           = primitive_msg.robot_id;
    double dest_x      = primitive_msg.parameters.at(0);
    double dest_y      = primitive_msg.parameters.at(1);
    dest               = Point(dest_x, dest_y);
    final_angle        = Angle::ofRadians(primitive_msg.parameters.at(2));
    rpm                = primitive_msg.parameters.at(3);
    small_kick_allowed = primitive_msg.extra_bits.at(0);
}


std::string DribblePrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int DribblePrimitive::getRobotId() const
{
    return robot_id;
}

Point DribblePrimitive::getDestination() const
{
    return dest;
}

Angle DribblePrimitive::getFinalAngle() const
{
    return final_angle;
}

double DribblePrimitive::getRpm() const
{
    return rpm;
}

bool DribblePrimitive::isSmallKickAllowed() const
{
    return small_kick_allowed;
}


std::vector<double> DribblePrimitive::getParameters() const
{
    std::vector<double> parameters = {dest.x(), dest.y(), final_angle.toRadians(), rpm};

    return parameters;
}

std::vector<bool> DribblePrimitive::getExtraBits() const
{
    std::vector<bool> extraBits = {small_kick_allowed};

    return extraBits;
}

void DribblePrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DribblePrimitive::operator==(const DribblePrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->final_angle == other.final_angle && this->rpm == other.rpm &&
           this->small_kick_allowed == other.small_kick_allowed;
}

bool DribblePrimitive::operator!=(const DribblePrimitive &other) const
{
    return !((*this) == other);
}
