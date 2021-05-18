#pragma once

#include "software/world/world.h"

// Transition conditions are conditions to check before transitioning between different
// gameplay behaviour

/**
 * Checks whether the robot is done moving
 *
 * @param robot The robot to check
 * @param destination The destination to move to
 * @param final_orientation The final orientation of the movement
 * @param DISTANCE_THRESHOLD distance from destination to be considered close enough
 * @param ANGLE_THRESHOLD angle from final orientation to be considered close enough
 *
 * @return true if robot movement is done
 */
bool robotReachedDestination(const Robot& robot, const Point& destination,
                             const Angle& final_orientation,
                             double DISTANCE_THRESHOLD    = 0.05,
                             const Angle& ANGLE_THRESHOLD = Angle::fromDegrees(2));

/**
 * Checks whether the robot is stopped
 *
 * @param robot The robot to check
 * @param SPEED_THRESHOLD The speed threshold at which robot is considered stopped
 *
 * @return if robot is stopped
 */
bool robotStopped(const Robot& robot, double SPEED_THRESHOLD = 0.05);

/**
 * Compares two points and returns true if within DISTANCE_THRESHOLD
 *
 * @param pt1 The first point
 * @param pt2 The second point
 * @param DISTANCE_THRESHOLD The threshold for how close the two points are
 */
bool comparePoints(const Point& pt1, const Point& pt2, double DISTANCE_THRESHOLD = 0.02);

/**
 * Compares two angles and returns true if within ANGLE_THRESHOLD
 *
 * @param angle1 The first angle
 * @param angle2 The second angle
 * @param ANGLE_THRESHOLD The threshold for how close the two angles are
 */
bool compareAngles(const Angle& angle1, const Angle& angle2,
                   const Angle& ANGLE_THRESHOLD = Angle::fromDegrees(2));
