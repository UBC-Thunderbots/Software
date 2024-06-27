#include "software/ai/hl/stp/tactic/transition_conditions.h"
#include "software/geom/algorithms/contains.h"
#include "proto/message_translation/tbots_protobuf.h" // TODO (NIMA): Remove
#include "software/logger/logger.h" // TODO (NIMA): Remove

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

bool isRobotReadyToChick(const Robot& robot, const Point& ball_position, const Angle& chick_direction)
{
    /*
     * Region behind ball where the robot should be to chick the ball.
     * Diagram not to scale!
     *    ┌─────────────┐
     *    │             │
     *    │             │
     *    │             │ behind_ball_region
     *    │             │
     *    │             │
     *    └──────┬──────┘
     *           ╎
     *          ─┼─ ball_position
     *           ╎
     *           ▼
     *    chick_direction
     */
    Vector behind_ball_direction =
            Vector::createFromAngle(chick_direction + Angle::half());
    Segment behind_ball_segment(ball_position + behind_ball_direction.normalize(DIST_TO_FRONT_OF_ROBOT_METERS),
                                ball_position + behind_ball_direction.normalize(2.5 * ROBOT_MAX_RADIUS_METERS));

    // The width of the kicker/chipper that the ball can be within to not hit the dribbler's side panels
    double chicker_width = robot.robotConstants().dribbler_width_meters - 2 * BALL_MAX_RADIUS_METERS;

    Polygon behind_ball_region = Polygon::fromSegment(behind_ball_segment, 0.0, chicker_width / 2);

    LOG(VISUALIZE) << *createDebugShapes({
             *createDebugShape(behind_ball_region, "behind_ball", "behind_ball")
     });
    return contains(behind_ball_region, robot.position()) &&
           compareAngles(robot.orientation(),
                         chick_direction, Angle::fromDegrees(5));
}