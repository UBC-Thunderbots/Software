#include "ai/primitive/primitive.h"

#include <exception>

thunderbots_msgs::Primitive Primitive::createMsg() const
{
    thunderbots_msgs::Primitive primitive_msg;

    primitive_msg.primitive_name = getPrimitiveName();
    primitive_msg.robot_id       = getRobotId();
    primitive_msg.parameters     = getParameters();
    // Boolean arrays can't be directly assigned, so we need
    // to use a loop
    for (auto data : getExtraBits())
    {
        primitive_msg.extra_bits.emplace_back(data);
    }

    return primitive_msg;
}

void Primitive::validatePrimitiveMessage(const thunderbots_msgs::Primitive& prim_msg,
                                         std::string prim_name) const
{
    if (prim_msg.primitive_name != prim_name)
    {
        throw std::invalid_argument("Primitive given (" + prim_msg.primitive_name +
                                    ") does not match expected name" + prim_name);
    }
}
