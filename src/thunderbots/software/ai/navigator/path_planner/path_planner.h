#pragma once
#include <optional>
#include <vector>
#include "geom/point.h"
#include "ai/navigator/obstacle/obstacle.h"
#include "ai/navigator/path_planner/violation_function.h"

class PathPlanner {
public:
    virtual std::optional <std::vector<Point>> findPath(
            const Point &start, const Point &dest,
            const std::vector <Obstacle> &obstacles,
            const ViolationFunction &violation_function) = 0;
    virtual ~PathPlanner() = default;
};