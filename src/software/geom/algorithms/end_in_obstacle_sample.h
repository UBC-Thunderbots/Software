#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

/**
 * Returns a point nearest to the the point provided; if the provided point is inside an obstacle, sampling will be
 * done around the point to find a nearby point that isn't inside an obstacle, and that point will be returned.
 * Otherwise, the originally provided point will just be returned
 *
 * @param obstacles a list of obstacles to test the point against
 * @param point the destination
 * @param navigable_area the entire region which samples cannot be outside of
 * @param initial_count the number of points to sample for the initial
 * @param rad_step the distance to increase the radius for each iteration of sampling
 * @param per_rad_step the number of samples to increase by for each iteration of sampling
 * @param range the max distance from given point that returned point is allowed to be
 * @return
 */
std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr>& obstacles,
                                         const Point& point,
                                         const Rectangle& navigable_area,
                                         int initial_count = 6, double radius_step = 0.15,
                                         int samples_per_radius_step = 2, double max_search_radius = 2.0);
