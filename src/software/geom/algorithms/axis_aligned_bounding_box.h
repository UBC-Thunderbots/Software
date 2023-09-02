#pragma once

#include "software/geom/circle.h"
#include "software/geom/rectangle.h"
#include "software/geom/polygon.h"

// TODO: Add tests

/**
 * Returns the axis-aligned bounding box of the given circle, inflated by the given
 *
 * @param shape The shape which the AABB of should be generated
 * @param inflation_radius Extran distance to add to the AABB in both dimensions
 * @return A rectangle which represents the AABB of the shape
 */
Rectangle axisAlignedBoundingBox(const Circle& circle, const double inflation_radius = 0);
Rectangle axisAlignedBoundingBox(const Rectangle& rectangle, const double inflation_radius = 0);
Rectangle axisAlignedBoundingBox(const Polygon& polygon, const double inflation_radius = 0);
