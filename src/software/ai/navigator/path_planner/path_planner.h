#pragma once
#include <variant>
#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/new_geom/point.h"
#include "software/geom/rectangle.h"
#include "software/geom/spline.h"

/**
 * PathPlanner is an interface for a path planner that,
 * given start and destination points, a list of obstacles,
 * will return the 'best' path from start to the destination
 * if a path exists, otherwise it will return nothing.
 */

using Path = std::optional<Spline>;

class PathPlanner
{
   public:
    /**
     * Returns a path between start and destination.
     * (consider the case where we have a "StraightLinePathPlanner" for testing purposes)
     *
     * @param start start point
     * @param destination destination point
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     *
     * @return a path between start and destination
     *     * no path is represented by a path of size 0
     */
    virtual Path findPath(const Point &start, const Point &destination,
                          const Rectangle &navigable_area,
                          const std::vector<Obstacle> &obstacles) = 0;

    virtual ~PathPlanner() = default;
};
