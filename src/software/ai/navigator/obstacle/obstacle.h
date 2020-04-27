#pragma once

#include <sstream>

#include "shared/constants.h"
#include "software/ai/navigator/obstacle/circle_obstacle.h"
#include "software/ai/navigator/obstacle/polygon_obstacle.h"
#include "software/ai/navigator/obstacle/shape_obstacle.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"

/**
 * An obstacle is an area to avoid for navigation
 */
using Obstacle = std::shared_ptr<ShapeObstacle>;

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle);
