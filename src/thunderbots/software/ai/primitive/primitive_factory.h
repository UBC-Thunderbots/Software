/**
 *  Declarations for functions to construct Primitives from various sources
 */
#pragma once

#include <memory>

#include "ai/primitive/primitive.h"
#include "thunderbots_msgs/Primitive.h"

namespace AI::Primitive
{
    /**
     * Given a ROS Primitive message, constructs a concrete Primitive object and returns
     * a unique_ptr using the Abstract Primitive interface. This acts like a Primitive
     * factory.
     *
     * @param primitive_msg the Primitive message from which to construct the Primitive
     * @throws std::invalid_argument if primitive is unknown
     * @return a unique_ptr to a Primitive object
     */
    std::unique_ptr<::Primitive> createPrimitiveFromROSMessage(
        const thunderbots_msgs::Primitive& primitive_msg);

}  // namespace AI::Primitive
