#include "software/ai/hl/stp/tactic/transition_conditions.h"

bool moveRobotDone(const Robot& robot, const Point& destination,
                   const Angle& final_orientation, double ROBOT_CLOSE_TO_DEST_THRESHOLD,
                   const Angle& ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD)
{
    return (robot.position() - destination).length() < ROBOT_CLOSE_TO_DEST_THRESHOLD &&
           (robot.orientation().minDiff(final_orientation) <
            ROBOT_CLOSE_TO_ORIENTATION_THRESHOLD);
}
