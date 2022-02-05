#include "software/geom/point.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/geom/linear_spline2d.h"
#include "enlsvg_path_planner.h"

#include <set>
#include <map>

using Path = LinearSpline2d;

class GlobalPathPlanner
{
    public:
        GlobalPathPlanner(const World &world);
        
        std::optional<Path> findPath(const Point &start, const Point &end, 
            const std::set<MotionConstraint> &motion_constraints);
        
    private:
        std::map<std::set<MotionConstraint>, std::unique_ptr<EnlsvgPathPlanner>> planners;
};