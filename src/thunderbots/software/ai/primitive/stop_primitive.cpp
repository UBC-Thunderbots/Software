#include "ai/primitive/stop_primitive.h"

const std::string StopPrimitive::PRIMITIVE_NAME = "Stop Primitive";

StopPrimitive::StopPrimitive(unsigned int robot_id, bool stop) : robot_id(robot_id), stop(stop)
{
}

StopPrimitive::StopPrimitive(const thunderbots_msgs::Primitive &primitive_msg)
{
    validatePrimitiveMessage(primitive_msg, getPrimitiveName());

    robot_id = primitive_msg.robot_id;
}

std:string StopPrimitive::getPrimitiveName() const
{
    return PRIMITIVE_NAME;
};

unsigned int StopPrimitive::getRobotId() const
{
    return robot_id;
}

std::vector<double> StopPrimitive::getParameterArray() const
{
    return std::vector<double>(0);
}

std::vector<bool> StopPrimitive::getExtraBitArray() const
{
    return std::vector<bool>(stop);
}