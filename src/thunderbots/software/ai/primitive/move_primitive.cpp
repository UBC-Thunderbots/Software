#include "ai/primitive/move_primitive.h"

MovePrimitive::MovePrimitive(
    unsigned int robot_id, const Point &dest, const Angle &final_angle,
    double final_speed)
    : robot_id(robot_id), dest(dest), final_angle(final_angle), final_speed(final_speed)
{
}

MovePrimitive::MovePrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    if (primitive_msg.primitive_name != getPrimitiveName())
    {
        // TODO: Throw a proper exception here
        std::cerr << "Error: Move Primitive constructed from wrong Primitive msg"
                  << std::endl;
        exit(1);
    }

    robot_id      = primitive_msg.robot_id;
    double dest_x = primitive_msg.parameters.at(0);
    double dest_y = primitive_msg.parameters.at(1);
    dest          = Point(dest_x, dest_y);
    final_angle   = Angle::ofRadians(primitive_msg.parameters.at(2));
    final_speed   = primitive_msg.parameters.at(3);
}


std::string MovePrimitive::getPrimitiveName() const
{
    return MOVE_PRIMITIVE_NAME;
}

unsigned int MovePrimitive::getRobotId() const
{
    return robot_id;
}

std::vector<double> MovePrimitive::getParameterArray() const
{
    std::vector<double> parameters = {dest.x(), dest.y(), final_angle.toRadians(),
                                      final_speed};

    return parameters;
}

std::vector<bool> MovePrimitive::getExtraBitArray() const
{
    return std::vector<bool>();
}
