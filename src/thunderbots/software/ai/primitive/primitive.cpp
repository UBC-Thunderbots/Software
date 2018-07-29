#include "ai/primitive/primitive.h"

thunderbots_msgs::Primitive Primitive::createMsg() const
{
    thunderbots_msgs::Primitive primitive_msg;

    primitive_msg.primitive_name = getPrimitiveName();
    primitive_msg.robot_id       = getRobotId();
    primitive_msg.parameters     = getParameterArray();
    // Boolean arrays can't be directly assigned, so we need
    // to use a loop
    for (auto data : getExtraBitArray())
    {
        primitive_msg.extra_bits.emplace_back(data);
    }

    return primitive_msg;
}
