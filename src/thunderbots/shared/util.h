#pragma once
#include <math.h>

typedef struct
{
    float x, y;
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
float dot2D(Vector2D first, Vector2D second);

/**
 * Find a point in a robot's local coordinates, where the origin is the
 * robot's position, the x-axis is along the orientation of the robot, and
 * the y-axis is perpendicular to the x-axis
 *
 * @param robotPosition the position of the robot
 *
 * @param point the point, in global coordinates, to be converted to robot-local
 * coordinates
 *
 * @param orientationAngle the orientation of the robot in radians, where 0 is direction
 * of global positive x-axis
 *
 * @return the input point, in coordinates relative to robotPosition and orientationAngle
 */
Vector2D toRobotLocalCoords(Vector2D robotPosition, Vector2D point,
                            float orientationAngle);