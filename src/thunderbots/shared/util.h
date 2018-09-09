#pragma once
#include <math.h>

typedef struct
{
    double x, y;
} Vector2D;

/**
 * Finds the dot product of two vectors.
 *
 * @param first a 2D vector
 *
 * @param second another 2D vector
 *
 * @return the dot product of first and second
 */
double dot2D(Vector2D first, Vector2D second);

/**
 * Find a point in a robot's local coordinates, where the origin is the
 * robot's position, the x-axis is along the orientation of the robot, and
 * the y-axis is perpendicular to the x-axis
 *
 * @param robot_position the position of the robot
 *
 * @param robot_orientation the orientation of the robot in radians, where 0 is direction
 * of global positive x-axis
 *
 * @param point the point, in global coordinates, to be converted to robot-local
 * coordinates
 *
 * @return the input point, in coordinates relative to robotPosition and orientationAngle
 */
Vector2D toRobotLocalCoords(Vector2D robot_position, double robot_orientation,
                            Vector2D point);