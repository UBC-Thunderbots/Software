#pragma once

#include <vector>

#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "extlibs/hrvo/agent.h"

/**
 * Set of functions that generate velocity obstacles relative to a shape
 */

Agent::VelocityObstacle generateVelocityObstacle(const Circle &circle, const Agent& agent);
Agent::VelocityObstacle generateVelocityObstacle(const Rectangle &rectangle, const Agent& agent);
Agent::VelocityObstacle generateVelocityObstacle(const Polygon &polygon, const Agent& agent);
