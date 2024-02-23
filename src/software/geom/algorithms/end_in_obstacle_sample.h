#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

/**
 * Returns a point nearest to the the point provided; if the provided point is inside an
 * obstacle, sampling will be done around the point to find a nearby point that isn't
 * inside an obstacle, and that point will be returned. Otherwise, the originally provided
 * point will just be returned
 *
 * @param obstacles a list of obstacles to test the point against
 * @param point the destination point
 * @param navigable_area the rectangular region which the returned point must be inside of
 * @param initial_count the number of points to sample for the initial radius in the case
 * where sampling is done
 * @param radius_step the distance to increase the radius for each iteration in the case
 * where sampling is done
 * @param samples_per_radius_step the number of samples to add for each iteration in the
 * case where sampling is done
 * @param max_search_radius the max distance from point that we are allowed to sample in
 * the case where sampling is done
 * @return if successful, the closest point to the provided point param that is not inside an obstacle
 */
std::optional<Point> endInObstacleSample(const std::vector<ObstaclePtr>& obstacles,
                                         const Point& point,
                                         const Rectangle& navigable_area,
                                         int initial_count = 6, double radius_step = 0.15,
                                         int samples_per_radius_step = 2,
                                         double max_search_radius    = 2.0);
