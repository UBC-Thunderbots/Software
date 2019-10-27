#pragma once
#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/ai/navigator/path_planner/path_planner.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/spline.h"

/**
 * PathManager is an interface for a path manager that,
 * given a list of start and destination point pairs,
 * a list of obstacles, and navigable_area will return
 * a list of 'best' paths between each of the pairs of
 * start and destination
 */

using Path = std::optional<Spline>;

struct PathObjective
{
    const Point start;
    const Point end;
};

class PathManager
{
   public:
    /**
     * Returns paths for each objective, given navigable_area and obstacles
     *
     * @param objectives vector of start and end points
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid, excluding the robots 
     *  whose paths are being planned
     *
     * @return a paths for each pair of start and end points
     */
    virtual std::vector<Path> getManagedPaths(std::vector<PathObjective> objectives,
                                              const Rectangle &navigable_area,
                                              const std::vector<Obstacle> &obstacles) = 0;

    virtual ~PathManager() = default;
};
