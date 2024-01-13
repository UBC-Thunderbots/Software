#pragma once

#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr> obstacles,
                          const Point& point,
                          const Rectangle& navigable_area,
                          int initial_count = 6,
                          double multiplier = 1.5,
                          double range = 2.0);
