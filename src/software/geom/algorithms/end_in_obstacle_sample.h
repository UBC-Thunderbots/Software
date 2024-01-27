#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr> obstacles,
                                         const Point& point,
                                         const Rectangle& navigable_area,
                                         int initial_count = 6, double rad_step = 0.15,
                                         int per_rad_step = 2, double range = 2.0);
