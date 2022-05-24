#include <gtest/gtest.h>

#include <vector>

#include "shared/robot_constants.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

namespace TestUtil
{
    /**
     * Given a list of points, we check that the path created by these bounds are within
     * the given bounding box.
     *
     * @param path_points    path formed by a vector of Points
     * @param bounding_box   a Rectangle representing the allowed navigable area for the
     * path
     */
    void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                           Rectangle bounding_box);

    /**
     * Given a list of points, we check that the path does not intersect any given
     * obstacles.
     *
     * @param path_points    path formed by a vector of Points
     * @param obstacles      obstacles to avoid
     */
    void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                           std::vector<ObstaclePtr> obstacles);

    /**
     * Given a list of points, we check that the path does not intersect any given
     * obstacles that are given as Polygons.
     *
     * The given Polygons are expanded by ROBOT_MAX_RADIUS_METERS internally.
     *
     * @param path_points    path formed by a vector of Points
     * @param obstacles      obstacles to avoid
     */
    void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                           std::vector<Polygon> obstacles);

	/**
	 * Given a list of points, print it out to the console. ONLY mean to be used as a debugging tool.
	 *
	 * @param path_points	path formed by a vector of Points
	 */
    void printPathPoints(std::vector<Point> path_points);
    
}  // namespace TestUtil
