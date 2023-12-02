#pragma once

#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"

/**
 * The primitive actions that a robot can perform
 */
class Primitive
{
   public:
    /**
     * Destructor
     */
    virtual ~Primitive() = default;

    /**
     * Gets the primitive proto message
     *
     * @return the primitive proto message
     */
    virtual std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const RobotNavigationObstacleFactory &obstacle_factory) = 0;

    /**
     * Gets the estimated cost of the primitive
     *
     * @return estimated cost of the primitive
     */
    double getEstimatedPrimitiveCost() const
    {
        return estimated_cost;
    }

   protected:
    double estimated_cost = 0;
};
