#pragma once

#include <optional>
#include <vector>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/spline.h"
#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"

/**
 * PathPlanner is an interface for a path planner that,
 * given start and destination points, a list of obstacles,
 * and navigable_area will return the 'best' path
 * from start to the destination
 */

using Path = Spline;

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
     *     * no path is represented by std::nullopt
     */
    virtual std::optional<Path> findPath(const Point &start, const Point &destination,
                                         const Rectangle &navigable_area,
                                         const std::vector<ObstaclePtr> &obstacles) = 0;

    virtual ~PathPlanner() = default;
};
