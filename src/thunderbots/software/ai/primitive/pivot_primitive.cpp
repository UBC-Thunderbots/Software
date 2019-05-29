#include "ai/primitive/pivot_primitive.h"

#include "ai/primitive/visitor/primitive_visitor.h"

const std::string PivotPrimitive::PRIMITIVE_NAME = "Pivot Primitive";

PivotPrimitive::PivotPrimitive(unsigned int robot_id, const Point &pivot_point,
                               const Angle &final_angle, const double pivot_radius)
    : robot_id(robot_id),
      pivot_point(pivot_point),
      final_angle(final_angle),
      pivot_radius(pivot_radius)
{
}

PivotPrimitive::PivotPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id        = primitive_msg.robot_id;
    double center_x = primitive_msg.parameters.at(0);
    double center_y = primitive_msg.parameters.at(1);
    pivot_point     = Point(center_x, center_y);
    final_angle     = Angle::ofRadians(primitive_msg.parameters.at(2));
    pivot_radius    = primitive_msg.parameters.at(3);
}

std::string PivotPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int PivotPrimitive::getRobotId() const
{
    return robot_id;
}

Point PivotPrimitive::getPivotPoint() const
{
    return pivot_point;
}

Angle PivotPrimitive::getFinalAngle() const
{
    return final_angle;
}

double PivotPrimitive::getPivotRadius() const
{
    return pivot_radius;
}

std::vector<double> PivotPrimitive::getParameters() const
{
    std::vector<double> parameters = {pivot_point.x(), pivot_point.y(),
                                      final_angle.toRadians(), pivot_radius};
    return parameters;
}

std::vector<bool> PivotPrimitive::getExtraBits() const
{
    return std::vector<bool>();
}

void PivotPrimitive::accept(PrimitiveVisitor &visitor) const
{
    visitor.visit(*this);
}

bool PivotPrimitive::operator==(const PivotPrimitive &other) const
{
    return this->robot_id == other.robot_id && this->pivot_point == other.pivot_point &&
           this->final_angle == other.final_angle &&
           this->pivot_radius == other.pivot_radius;
}

bool PivotPrimitive::operator!=(const PivotPrimitive &other) const
{
    return !((*this) == other);
}
