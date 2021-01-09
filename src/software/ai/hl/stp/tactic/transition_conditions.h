#pragma once

#include "software/world/world.h"

/**
 * Returns whether the robot is done moving
 *
 * @param robot The robot to check
 * @param destination The destination to move to
 * @param final_orientation The final orientation of the movement
 */
bool moveRobotDone(
    const Robot& robot, const Point& destination, const Angle& final_orientation,
    double ROBOT_CLOSE_TO_DEST_THRESHOLD              = 0.02,
    const Angle& ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD = Angle::fromDegrees(2));
