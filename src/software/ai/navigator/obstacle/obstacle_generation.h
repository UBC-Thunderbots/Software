#pragma once

#include "software/ai/intent/avoid_area.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/point.h"
#include "software/geom/util.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/world.h"

/**
 * Create obstacles for the given avoid areas, with a buffer such that the edge
 * of the robot does not protrude into the area
 *
 * @param avoid_areas The areas to convert into obstacles
 * @param world the world that's full of obstacles
 *
 * @return Obstacles representing the given avoid areas
 */
std::vector<Obstacle> getObstaclesFromAvoidAreas(
    const std::vector<AvoidArea> &avoid_areas, World world);

/**
 * Get Obstacles from a Team
 *
 * @return vector of obstacles
 */
std::vector<Obstacle> getObstaclesFromTeam(const Team &team);
