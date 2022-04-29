#pragma once

#include <vector>

#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "extlibs/hrvo/agent.h"

/**
 * Set of functions that generate velocity obstacles for a static obstacle
 */

Agent::VelocityObstacle generateVelocityObstacle(const Circle &robot, const Circle& obstacle);
Agent::VelocityObstacle generateVelocityObstacle(const Circle &robot, const Polygon &obstacle);
