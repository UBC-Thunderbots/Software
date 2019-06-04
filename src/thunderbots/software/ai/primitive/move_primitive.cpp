#include "ai/primitive/move_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string MovePrimitive::PRIMITIVE_NAME = "Move Primitive";

MovePrimitive::MovePrimitive(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double final_speed,
                             bool enable_dribbler, bool enable_autokick)
    : robot_id(robot_id),
      dest(dest),
      final_angle(final_angle),
      final_speed(final_speed),
      enable_dribbler(enable_dribbler),
      enable_autokick(enable_autokick)
{
}

MovePrimitive::MovePrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id        = primitive_msg.robot_id;
    double dest_x   = primitive_msg.parameters.at(0);
    double dest_y   = primitive_msg.parameters.at(1);
    dest            = Point(dest_x, dest_y);
    final_angle     = Angle::ofRadians(primitive_msg.parameters.at(2));
    final_speed     = primitive_msg.parameters.at(3);
    enable_dribbler = static_cast<bool>(primitive_msg.parameters.at(4));
    enable_autokick = static_cast<bool>(primitive_msg.parameters.at(5));
}


std::string MovePrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int MovePrimitive::getRobotId() const
{
    return robot_id;
}

Point MovePrimitive::getDestination() const
{
    return dest;
}

Angle MovePrimitive::getFinalAngle() const
{
    return final_angle;
}

double MovePrimitive::getFinalSpeed() const
{
    return final_speed;
}

bool MovePrimitive::isAutoKickEnabled() const
{
    return enable_autokick;
}

bool MovePrimitive::isDribblerEnabled() const
{
    return enable_dribbler;
}

std::vector<double> MovePrimitive::getParameters() const
{
    std::vector<double> parameters = {dest.x(),
                                      dest.y(),
                                      final_angle.toRadians(),
                                      final_speed,
                                      (double)enable_dribbler,
                                      (double)enable_autokick};

    return parameters;
}

std::vector<bool> MovePrimitive::getExtraBits() const
{
    return std::vector<bool>();
}

void MovePrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MovePrimitive::operator==(const MovePrimitive &other) const
{
    return this->robot_id == other.robot_id && this->dest == other.dest &&
           this->final_angle == other.final_angle &&
           this->final_speed == other.final_speed &&
           this->enable_dribbler == other.enable_dribbler &&
           this->enable_autokick == other.enable_autokick;
}

bool MovePrimitive::operator!=(const MovePrimitive &other) const
{
    return !((*this) == other);
}
