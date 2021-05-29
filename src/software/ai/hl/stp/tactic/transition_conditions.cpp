#include "software/ai/hl/stp/tactic/transition_conditions.h"

bool robotReachedDestination(const Robot& robot, const Point& destination,
                             const Angle& final_orientation, double DISTANCE_THRESHOLD,
                             const Angle& ANGLE_THRESHOLD)
{
    return comparePoints(robot.position(), destination, DISTANCE_THRESHOLD) &&
           compareAngles(robot.orientation(), final_orientation, ANGLE_THRESHOLD);
}

bool robotStopped(const Robot& robot, double SPEED_THRESHOLD)
{
    return robot.velocity().length() <= SPEED_THRESHOLD;
}

bool comparePoints(const Point& pt1, const Point& pt2, double DISTANCE_THRESHOLD)
{
    return (pt1 - pt2).length() <= DISTANCE_THRESHOLD;
}

bool compareAngles(const Angle& angle1, const Angle& angle2, const Angle& ANGLE_THRESHOLD)
{
    return angle1.minDiff(angle2) <= ANGLE_THRESHOLD;
}
