/**
 *  Declarations for functions to construct Primitives from various sources
 */
#pragma once

#include <memory>
#include "thunderbots_msgs/Primitive.h"
#include "ai/primitive/primitive.h"

namespace AI::Primitive {

    /**
     * Creates a Primitive from a given ROS message
     *
     * @param primitive_msg The Primitive ROS message
     *
     * @return The primitive class represented by the ROS message
     */
    std::unique_ptr<::Primitive> createPrimitiveFromROSMessage(
            const thunderbots_msgs::Primitive& primitive_msg);

}