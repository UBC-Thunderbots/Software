#pragma once

#include "software/geom/circle.h"
#include "software/world/world.h"
#include "software/geom/rectangle.h"

/**
 * Finds good points to chip the ball to
 *
 * @param world The world. We assume the ball is being chipped from its current
 * position
 *
 * @return a vector of circles where the center is a good point to chip to, and the
 *         radius is the distance to the nearest enemy
 */
std::vector<Circle> findGoodChipTargets(const World& world, std::optional<Rectangle> target_area_rectangle = std::nullopt);
