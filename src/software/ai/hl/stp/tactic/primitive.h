#pragma once

#include "proto/primitive.pb.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/trajectory/trajectory_path.h"

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
     * @param world Current state of the world
     * @param motion_constraints Motion constraints to consider
     * @param obstacle_factory Obstacle factory to use for generating obstacles
     * @return the primitive proto message
     */
    virtual std::pair<std::optional<TrajectoryPath>,
                      std::unique_ptr<TbotsProto::Primitive>>
    generatePrimitiveProtoMessage(
        const World &world,
        const std::set<TbotsProto::MotionConstraint> &motion_constraints,
        const std::map<RobotId, TrajectoryPath> &robot_trajectories,
        const RobotNavigationObstacleFactory &obstacle_factory) = 0;

    /**
     * Fill the obstacle list and path visualization with the obstacles and path
     * of this primitive
     *
     * @param obstacle_list_out Reference to the ObstacleList proto to add obstacles to
     * @param path_visualization_out Reference to the PathVisualization proto to add path
     */
    virtual void getVisualizationProtos(
        TbotsProto::ObstacleList &obstacle_list_out,
        TbotsProto::PathVisualization &path_visualization_out) const = 0;

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
