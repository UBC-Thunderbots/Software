#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "shared/robot_constants.h"

#include <gtest/gtest.h>
#include <vector>

namespace TestUtil
{
    void checkPathDoesNotExceedBoundingBox(std::vector<Point> path_points,
                                           Rectangle bounding_box);

    void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                           std::vector<ObstaclePtr> obstacles);

    void checkPathDoesNotIntersectObstacle(std::vector<Point> path_points,
                                           std::vector<Polygon> obstacles);
}