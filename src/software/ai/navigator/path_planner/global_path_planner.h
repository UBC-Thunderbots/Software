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

class GlobalPathPlanner
{
    public:
        GlobalPathPlanner(const std::shared_ptr<const RobotNavigationObstacleConfig> navigation_obstacle_config, const World &world, 
                                     const Rectangle &navigable_area);        
        std::shared_ptr<EnlsvgPathPlanner> getPathGenerator(const World &world, const std::set<MotionConstraint> constraints);
        
    private:
        std::map<std::set<MotionConstraint>, std::shared_ptr<EnlsvgPathPlanner>> planners;
};