#pragma once

#include "software/new_geom/circle.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"

/**
 * Returns whether `container` contains `contained`
 *
 * @param container
 * @param contained
 *
 * @return whether `container` contains `contained`
 */
bool contains(const Circle& container, const Segment& contains);
bool contains(const Circle& container, const Point& contained);
bool contains(const Polygon& container, const Point& contained);
bool contains(const Ray& container, const Point& contained);
bool contains(const Segment& container, const Point& contained,
              double fixed_epsilon = FIXED_EPSILON, int ulps_distance = ULPS_EPSILON_TEN);
bool contains(const Rectangle& container, const Point& contained);
