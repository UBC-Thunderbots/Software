#include "ai/primitive/pivot_primitive.h"

const std::string PivotPrimitive::PRIMITIVE_NAME = "Pivot Primitive";

PivotPrimitive::PivotPrimitive(unsigned int robot_id,
                               double center_x,
                               double center_y,
                               Angle &final_angle,
                               Angle &robot_orientation)
    : robot_id(robot_id),
      center_x(center_x),
      center_y(center_y),
      final_angle(final_angle),
      robot_orientation(robot_orientation)
{
}


PivotPrimitive::PivotPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id          = primitive_msg.robot.id;
    center_x          = primitive_msg.parameters.at(0);
    center_y          = primitive_msg.parameters.at(1);
    final_angle       = Angle::ofRadians(primitive_msg.parameters.at(2));
    robot_orientation = Angle::ofRadians(primitive_msg.parameters.at(3));
}


std::string PivotPrimitive::getPrimitiveName()
{
    return PRIMITIVE_NAME;
}


unsigned int PivotPrimitive::getRobotId()
{
    return robot_id;
}

std::vector<double> getParameterArray()
{
    std::vector<double> parameters = {center_x,
                                      center_y,
                                      final_angle.toRadians(),
                                      robot_orientation.toRadians()};
    return parameters;
}


std::vector<bool> getExtraBitArray()
{
    return std::vector<bool>();
}