#pragma once

#include <vector>

#include "software/geom/circle.h"
#include "software/geom/polygon.h"
// TODO: Added for VO import (update this and BUILD when #2589 is merged)
#include "extlibs/hrvo/velocity_obstacle.h"

/**
 * Set of functions that generate velocity obstacles for a static obstacle
 */

VelocityObstacle generateVelocityObstacle(const Circle& obstacle, const Circle &robot);
VelocityObstacle generateVelocityObstacle(const Polygon &obstacle, const Circle &robot);
