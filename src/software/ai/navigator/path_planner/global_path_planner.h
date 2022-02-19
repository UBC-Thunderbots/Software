#include "software/geom/point.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/geom/linear_spline2d.h"
#include "enlsvg_path_planner.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/motion_constraint/motion_constraint_set_builder.h"

#include "software/logger/logger.h"

#include <set>
#include <map>

using Path = LinearSpline2d;

/**
* GlobalPathPlanner is a module that constructs every possible path planner for every possible combination of static 
* obstacles. It pre-computes static obstacle planners in order to achieve path planning efficiencies during the game.
*
* The GlobalPathPlanner uses MotionConstraints to create these static obstacles.
*/
class GlobalPathPlanner
{
    public:
        /**
        * Creates path planners for every possible combination of obstacles using the given parameters.
        *
        * @param navigation_obstacle_config the config is used to convert motion constraints into obstacles
        * @param world                      the world is used to create the path planner gand obstacles
        * @param navigable_area             the path planner's navigable area
        */
        GlobalPathPlanner(const std::shared_ptr<const RobotNavigationObstacleConfig> navigation_obstacle_config, 
                            const World &world, 
                            const Rectangle &navigable_area);    
                            
        /**
        * Given a set of motion constraints, returns the relevant EnlsvgPathPlanner. If the path planner is not cached,
        * then it returns a path planner without any static obstacles.
        *
        * @param constraints    the motion constraints used to get the relevant path planner
        *
        * @return path planner for static obstacles created by constraints
        */
        std::shared_ptr<EnlsvgPathPlanner> getPathGenerator(const std::set<MotionConstraint> constraints);
        
    private:
        std::map<std::set<MotionConstraint>, std::shared_ptr<EnlsvgPathPlanner>> planners;
};