#include "ai/primitive/pivot_primitive.h"

const std::string PivotPrimitive::PRIMITIVE_NAME = "Pivot Primitive";

PivotPrimitive::PivotPrimitive(unsigned int robot_id, const Point &pivot_point,
                               const Angle &final_angle, const Angle &robot_orientation)
    : robot_id(robot_id),
      pivot_point(pivot_point),
      final_angle(final_angle),
      robot_orientation(robot_orientation)
{
}

PivotPrimitive::PivotPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id          = primitive_msg.robot_id;
    double center_x   = primitive_msg.parameters.at(0);
    double center_y   = primitive_msg.parameters.at(1);
    pivot_point       = Point(center_x, center_y);
    final_angle       = Angle::ofRadians(primitive_msg.parameters.at(2));
    robot_orientation = Angle::ofRadians(primitive_msg.parameters.at(3));
}

std::string PivotPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

PrimitiveType PivotPrimitive::getPrimitiveType() const
{
    return PrimitiveType::PIVOT;
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

Angle PivotPrimitive::getRobotOrientation() const
{
    return robot_orientation;
}

std::vector<double> PivotPrimitive::getParameters() const
{
    std::vector<double> parameters = {pivot_point.x(), pivot_point.y(),
                                      final_angle.toRadians(),
                                      robot_orientation.toRadians()};
    return parameters;
}

std::vector<bool> PivotPrimitive::getExtraBits() const
{
    return std::vector<bool>();
}
