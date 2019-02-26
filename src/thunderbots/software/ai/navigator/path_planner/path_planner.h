#pragma once
#include <vector>
#include <optional>
#include "geom/point.h"

class PathPlanner {
public:
    virtual std::optional<std::vector<Point>>
        findPath(const Point& start, const Point& dest) = 0;
    virtual ~PathPlanner() = 0;
};