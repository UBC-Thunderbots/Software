#pragma once

#include "software/geom/circle.h"
#include "software/world/world.h"

/**
 * Finds good points to chip the ball to within the specified target area
 *
 * @param world the world; we assume the ball is being chipped from its current
 * position
 * @param target_area the area on the field that chip targets should be restrained to
 *
 * @return a vector of circles where the center is a good point to chip to, and the
 *         radius is the distance to the nearest enemy
 */
std::vector<Circle> findGoodChipTargets(const World& world,
                                        const Rectangle& target_area);

/**
 * Finds good points to chip the ball to
 *
 * @param world The world. We assume the ball is being chipped from its current
 * position
 *
 * @return a vector of circles where the center is a good point to chip to, and the
 *         radius is the distance to the nearest enemy
 */
std::vector<Circle> findGoodChipTargets(const World& world);
