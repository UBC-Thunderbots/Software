#include <map>
#include <set>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"
#include "software/geom/linear_spline2d.h"
#include "software/geom/point.h"
#include "software/logger/logger.h"

// TODO #2504: The GlobalPathPlanner doesn't handle
// MotionConstraint::AVOID_BALL_PLACEMENT_INTERFERENCE and
// MotionConstraint::HALF_METER_AROUND_BALL correctly in most situations
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
     * Creates path planners for every possible combination of obstacles using the World´s
     * field and obstacle config's motion constraints.
     *
     * @param navigation_obstacle_config the config used to get motion constraints
     * into obstacles
     * @param world                      the world used to create the path planner grid
     * and obstacles
     */
    GlobalPathPlannerFactory(const std::shared_ptr<const RobotNavigationObstacleConfig>
                                 navigation_obstacle_config,
                             const World &world);

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
        const std::set<MotionConstraint> &constraints) const;

   private:
    std::map<std::set<MotionConstraint>, std::shared_ptr<const EnlsvgPathPlanner>>
        planners;
};
