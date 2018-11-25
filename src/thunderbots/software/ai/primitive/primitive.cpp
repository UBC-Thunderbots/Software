#include "ai/primitive/primitive.h"

#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/move_primitive.h"
#include <exception>

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

std::unique_ptr<Primitive> Primitive::createPrimitive(
    const thunderbots_msgs::Primitive& primitive_msg)
{
    std::unique_ptr<Primitive> prim_ptr;

    if (primitive_msg.primitive_name == MovePrimitive::PRIMITIVE_NAME)
    { 
        prim_ptr = std::make_unique<MovePrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == DirectVelocityPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<DirectVelocityPrimitive>(primitive_msg);
    }
    else
    {
        throw std::invalid_argument("Error: Unknown Primitive (" + primitive_msg.primitive_name + ") ");
    }

    return prim_ptr;
}

void Primitive::validatePrimitiveMessage(const thunderbots_msgs::Primitive& prim_msg,
                                         std::string prim_name) const
{
    if (prim_msg.primitive_name != prim_name)
    {   
        throw std::invalid_argument("Primitive given (" + prim_msg.primitive_name + ") does not match expected name" + prim_name);  
    }
}
