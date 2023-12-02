#pragma once

#include <memory>

#include "proto/primitive.pb.h"
#include "software/ai/hl/stp/tactic/primitive.h"

class StopPrimitive : public Primitive
{
   public:
    StopPrimitive()           = default;
    ~StopPrimitive() override = default;

    /**
     * Gets the primitive proto message
     *
     * @return the primitive proto message
     */
    std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const RobotNavigationObstacleFactory &obstacle_factory) override;
};
