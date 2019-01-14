#include "ai/primitive/primitive.h"

#include "ai/primitive/catch_primitive.h"
#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/pivot_primitive.h"

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

std::unique_ptr<Primitive> Primitive::createPrimitive(
    const thunderbots_msgs::Primitive& primitive_msg)
{
    std::unique_ptr<Primitive> prim_ptr;

    if (primitive_msg.primitive_name == MovePrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<MovePrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == CatchPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<CatchPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == ChipPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<ChipPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == DirectVelocityPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<DirectVelocityPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == KickPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<KickPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == PivotPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<PivotPrimitive>(primitive_msg);
    }
    else
    {
        // TODO: Throw unknown primitive exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
        std::cerr << "Error: Unexpected Primitive message of type "
                  << primitive_msg.primitive_name << std::endl;
        std::exit(1);
    }

    return prim_ptr;
}

void Primitive::validatePrimitiveMessage(const thunderbots_msgs::Primitive& prim_msg,
                                         std::string prim_name) const
{
    if (prim_msg.primitive_name != prim_name)
    {
        // TODO: Throw a proper exception here
        // https://github.com/UBC-Thunderbots/Software/issues/16
        std::cerr << "Error: Move Primitive constructed from wrong Primitive msg"
                  << std::endl;
        exit(1);
    }
}
