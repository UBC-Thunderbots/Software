#pragma once

#include <optional>

#include "software/geom/rectangle.h"

/**
 * Steps a given distance along the perimeter of the rectangle, starting from  
 * a given point on the rectangle's perimeter.
 * 
 * @param rectangle the rectangle
 * @param start the starting point along the perimeter of the rectangle to step from
 * @param distance the distance to step along the perimeter of the rectangle. Positive
 * for clockwise, negative for counterclockwise
 *
 * @return the point along the perimeter of the rectangle after stepping the 
 * specified distance, if the provided starting point was along the perimeter of the
 * rectangle 
 */
std::optional<Point> stepAlongPerimeter(const Rectangle &rectangle, 
                                        const Point &start, 
                                        double distance);

/**
 * Returns the edge of the rectangle on which the given point lies
 *
 * @param rectangle the rectangle
 * @param point a point along the perimeter of the rectangle
 *
 * @return the edge of the rectangle on which the given point lies,
 * if the point lies along the perimeter of the rectangle
 */
std::optional<Segment> edgeContainingPoint(const Rectangle &rectangle, const Point &point);