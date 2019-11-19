#pragma once

#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/util.h"
#include "software/new_geom/point.h"
#include "software/util/parameter/dynamic_parameters.h"
#include "software/world/world.h"

/**
 * Create obstacles for the given motion constraints
 *
 * @param motion_constraints
 * @param world world to create obstacles for
 *
 * @return obstacles
 */
std::vector<Obstacle> getObstaclesFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world);

/**
 * Get Obstacles from a Team
 *
 * @param team team to get obstacles from
 *
 * @return vector of obstacles
 */
std::vector<Obstacle> getObstaclesFromTeam(const Team &team);
