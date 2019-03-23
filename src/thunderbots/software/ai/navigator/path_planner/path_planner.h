#pragma once
#include <optional>
#include <vector>

#include "ai/navigator/obstacle/obstacle.h"
#include "ai/navigator/path_planner/violation_function.h"
#include "geom/point.h"

class PathPlanner
{
   public:
    /**
     * Returns a path from start to dest given obstacles and a
     * violation, otherwise returns std::nullopt if a path is
     * not found.
     *
     * @param start start point
     * @param dest destination point
     * @param obstacles a vector of obstacles
     * @param violation_function a function that returns the distance
     *                          that a point is violating a boundary by
     * @return
     */
    virtual std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
        const ViolationFunction &violation_function) = 0;
    virtual ~PathPlanner()                           = default;
};