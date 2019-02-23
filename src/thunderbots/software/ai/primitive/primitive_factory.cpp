/**
 *  Implementations for functions to construct Primitives from various sources
 */

#include "ai/primitive/primitive_factory.h"

#include <exception>

#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/direct_wheels_primitive.h"
#include "ai/primitive/dribble_primitive.h"
#include "ai/primitive/grsim_command_primitive_visitor_catch.h"
#include "ai/primitive/grsim_command_primitive_visitor_movespin.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/pivot_primitive.h"
#include "ai/primitive/stop_primitive.h"


std::unique_ptr<::Primitive> AI::Primitive::createPrimitiveFromROSMessage(
    const thunderbots_msgs::Primitive& primitive_msg)
{
    std::unique_ptr<::Primitive> prim_ptr;

    if (primitive_msg.primitive_name == MovePrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<MovePrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == MoveSpinPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<MoveSpinPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == DirectWheelsPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<DirectWheelsPrimitive>(primitive_msg);
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
    else if (primitive_msg.primitive_name == DribblePrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<DribblePrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == PivotPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<PivotPrimitive>(primitive_msg);
    }
    else if (primitive_msg.primitive_name == StopPrimitive::PRIMITIVE_NAME)
    {
        prim_ptr = std::make_unique<StopPrimitive>(primitive_msg);
    }
    else
    {
        throw std::invalid_argument("Error: Unknown Primitive (" +
                                    primitive_msg.primitive_name + ") ");
    }

    return prim_ptr;
}
