#include "software/ai/hl/stp/tactic/stop_primitive.h"

std::pair<std::optional<TrajectoryPath>, std::unique_ptr<TbotsProto::Primitive>>
StopPrimitive::generatePrimitiveProtoMessage(
    const WorldPtr &world_ptr, const std::set<TbotsProto::MotionConstraint> &motion_constraints,
    const std::map<RobotId, TrajectoryPath> &robot_trajectories,
    const RobotNavigationObstacleFactory &obstacle_factory)
{
    auto stop_primitive_msg = std::make_unique<TbotsProto::Primitive>();
    stop_primitive_msg->mutable_stop();
    return std::make_pair(std::nullopt, std::move(stop_primitive_msg));
}

void StopPrimitive::getVisualizationProtos(
    TbotsProto::ObstacleList &obstacle_list_out,
    TbotsProto::PathVisualization &path_visualization_out) const
{
}
