#include "ai/primitive/primitive.h"
#include "ai/primitive/move_primitive.h"

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
    const thunderbots_msgs::Primitive &primitive_msg)
{
    std::unique_ptr<Primitive> prim_ptr;

    if (primitive_msg.primitive_name == MOVE_PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<MovePrimitive>(primitive_msg);
    }
    else
    {
        // TODO: Throw unknown primitive exception here
        std::cerr << "Error: Unexpected Primitive message of type "
                  << primitive_msg.primitive_name << std::endl;
        std::exit(1);
    }

    return prim_ptr;
}
