#pragma once
#include <set>
#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_manager/path_objective.h"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/spline.h"

/**
 * PathManager is an interface for a path manager that,
 * given a list of start and destination point pairs,
 * a list of static obstacles, and navigable_area will return
 * a list of 'best' paths between each of the pairs of
 * start and destination
 */

class PathManager
{
   public:
    /**
     * Returns map of path objectives to paths guided by path objectives,
     * given navigable_area and static_obstacles
     *
     * @param objectives vector of path objectives
     * @param navigable_area Rectangle representing the navigable area
     * @param static_obstacles static obstacles to avoid, excluding the robots
     *  whose paths are being planned
     *
     * @return vector of paths
     */
    virtual const std::map<PathObjective, Path> getManagedPaths(
        const std::set<PathObjective> &objectives, const Rectangle &navigable_area,
        const std::vector<Obstacle> &static_obstacles) = 0;

    virtual ~PathManager() = default;
};
