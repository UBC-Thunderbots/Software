#pragma once

#include <map>
#include <set>

#include "proto/primitive.pb.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"
#include "software/geom/linear_spline2d.h"
#include "software/geom/point.h"
#include "software/logger/logger.h"

using Path = LinearSpline2d;

/**
 * GlobalPathPlannerFactory is a module that constructs every possible path planner for
 * every possible combination of static obstacles. It pre-computes static obstacle
 * planners in order to achieve path planning efficiencies during the game.
 *
 * The GlobalPathPlannerFactory uses MotionConstraints to create these static obstacles.
 */
class GlobalPathPlannerFactory
{
   public:
    /**
     * Creates path planners for every possible combination of obstacles using the field,
     * obstacle config, and motion constraints.
     *
     * @param navigation_obstacle_config the config used to get motion constraints
     * into obstacles
     * @param field the field used to create the path planner grid and obstacles
     */
    GlobalPathPlannerFactory(const std::shared_ptr<const RobotNavigationObstacleConfig>
                                 navigation_obstacle_config,
                             const Field &field);

    /**
     * Given a set of motion constraints, returns the relevant EnlsvgPathPlanner. If the
     * path planner is not cached, then it returns a path planner without any static
     * obstacles.
     *
     * @param constraints    the motion constraints used to get the relevant path planner
     *
     * @return path planner for static obstacles created by constraints
     */
    std::shared_ptr<const EnlsvgPathPlanner> getPathPlanner(
        std::set<TbotsProto::MotionConstraint> constraints) const;

    /**
     * Given a set of motion constraints, returns the relevant obstacles. If the
     * obstacles not cached, then it returns no obstacles.
     *
     * @param constraints    the motion constraints used to get the relevant obstacles
     *
     * @return obstacles for the motion constraints
     */
    google::protobuf::RepeatedPtrField<TbotsProto::Obstacles> getStaticObstacles(
        std::set<TbotsProto::MotionConstraint> constraints) const;

   private:
    std::map<std::set<TbotsProto::MotionConstraint>,
             std::shared_ptr<const EnlsvgPathPlanner>>
        planners;

    std::map<std::set<TbotsProto::MotionConstraint>,
             google::protobuf::RepeatedPtrField<TbotsProto::Obstacles>>
        motion_constraint_to_obstacles;

    // Cache which motion constraints are dynamic for convenience
    std::set<TbotsProto::MotionConstraint> dynamic_motion_constraints;
};
