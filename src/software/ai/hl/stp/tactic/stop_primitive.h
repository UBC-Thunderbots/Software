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
     * @param world Current state of the world
     * @param motion_constraints Motion constraints to consider
     * @param obstacle_factory Obstacle factory to use for generating obstacles
     * @return the primitive proto message
     */
    std::pair<std::optional<TrajectoryPath>, std::unique_ptr<TbotsProto::Primitive>> generatePrimitiveProtoMessage(
            const World &world, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
            const std::map<RobotId, TrajectoryPath> &robot_trajectories,
            const RobotNavigationObstacleFactory &obstacle_factory) override;

    /**
     * Fill the obstacle list and path visualization with the obstacles and path
     * of this primitive
     *
     * @param obstacle_list_out Reference to the ObstacleList proto to add obstacles to
     * @param path_visualization_out Reference to the PathVisualization proto to add path
     */
    void getVisualizationProtos(
        TbotsProto::ObstacleList &obstacle_list_out,
        TbotsProto::PathVisualization &path_visualization_out) const override;
};
