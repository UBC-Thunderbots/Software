#include "software/ai/hl/stp/tactic/transition_conditions.h"

bool moveRobotDone(const Robot& robot, const Point& destination,
                   const Angle& final_orientation, double DISTANCE_THRESHOLD,
                   const Angle& ANGLE_THRESHOLD)
{
    return (robot.position() - destination).length() < DISTANCE_THRESHOLD &&
           (robot.orientation().minDiff(final_orientation) < ANGLE_THRESHOLD);
}
