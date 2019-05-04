#pragma once

#include "ai/primitive/move_primitive.h"
#include "geom/point.h"
#include "geom/util.h"

/**
 * Calculates the transition speed for the robot between two line segments
 *
 * Calculates the speed that the robot should be at when it is at the end of a
 * given line segment in order to smoothly transition to another given line segment,
 * given a final speed at the end of the two line segments
 *
 * @param p1, p2, p3 are 3 points that define two line segments that form a path
 * @param final_speed is the intended final speed at the end of the path
 * @return the first segment's final speed after travelling from p1 to p2
 * for a smooth transition to the p2 to p3 path, scaled by the final speed at the end
 * of the path
 */
double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed);

/**
 * Takes a vector of points that represents a path and converts them to a vector of move
 * primitives
 *
 * @param robot_id the robot_id to assign the move primitives and path to
 * @param points the vector of points representing a path
 * @return a vector of move primitives converted from a vector of points
 */
std::vector<MovePrimitive> convertToMovePrimitives(unsigned int robot_id,
                                                   const std::vector<Point> &points);
