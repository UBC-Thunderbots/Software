#include "ai/primitive/shoot_primitive.h"

const std::string ShootPrimitive::PRIMITIVE_NAME = "Shoot Primitive";

ShootPrimitive::ShootPrimitive(unsigned int robot_id, const Point &shot_origin,
                           const Angle &shot_direction, double power, bool chip)
    : robot_id(robot_id), shot_origin(shot_origin), shot_direction(shot_direction), 
    power(power), chip(chip)
{
}

ShootPrimitive::ShootPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id             = primitive_msg.robot_id;
    double shot_origin_x = primitive_msg.parameters.at(0);
    double shot_origin_y = primitive_msg.parameters.at(1);
    shot_origin          = Point(shot_origin_x, shot_origin_y);
    shot_direction       = Angle::ofRadians(primitive_msg.parameters.at(2));
    power                = primitive_msg.parameters.at(3);
    chip                 = primitive_msg.extra_bits.at(0);
}


std::string ShootPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
}

unsigned int ShootPrimitive::getRobotId() const
{
    return robot_id;
}

std::vector<double> ShootPrimitive::getParameterArray() const
{
    std::vector<double> parameters = {shot_origin.x(), shot_origin.y(), 
                                      shot_direction.toRadians(), power};

    return parameters;
}

std::vector<bool> ShootPrimitive::getExtraBitArray() const
{
    std::vector<bool> extra_bits = {chip};

    return extra_bits;
}
