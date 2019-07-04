#include "ai/hl/stp/evaluation/robot.h"

#include <shared/constants.h>

#include "util/parameter/dynamic_parameters.h"



bool Evaluation::robotOrientationWithinAngleThresholdOfTarget(const Point position,
                                                              const Angle orientation,
                                                              const Point target,
                                                              Angle threshold)
{
    // Calculate the target orientation, then calculate the difference between facing
    // orientation and target orientation. The difference in angle will be in the range of
    // [0, pi]. Return true if the difference is smaller than threshold, false otherwise
    Angle target_orientation = (target - position).orientation();
    Angle diff_orientation   = orientation.minDiff(target_orientation);
    return diff_orientation < threshold;
}

bool Evaluation::robotHasPossession(Ball ball, Robot robot, Timestamp timestamp)
{
    // copied almost verbatim from legacy code
    Point robot_pos_at_time;
    Angle robot_ori_at_time;
    Point ball_pos_at_time;


    if (robot.getHistoryIndexFromTimestamp(timestamp))
    {
        robot_pos_at_time = robot.getPreviousPositions().at(
            *robot.getHistoryIndexFromTimestamp(timestamp));
        robot_ori_at_time = robot.getPreviousOrientations().at(
            *robot.getHistoryIndexFromTimestamp(timestamp));
    }
    else
    {
        robot_pos_at_time = robot.position();
        robot_ori_at_time = robot.orientation();
    }

    if (ball.getHistoryIndexFromTimestamp(timestamp))
    {
        ball_pos_at_time =
            ball.getPreviousPositions().at(*ball.getHistoryIndexFromTimestamp(timestamp));
    }
    else
    {
        ball_pos_at_time = ball.position();
    }


    // check if the ball is within a certain distance of the robot
    auto max_dist_to_robot =
        ROBOT_MAX_RADIUS_METERS +
        Util::DynamicParameters::Evaluation::Possession::possession_dist.value();
    if ((ball_pos_at_time - robot_pos_at_time).len() > max_dist_to_robot)
    {
        return false;
    }
    else
    {
        // check that ball is in a 90-degree cone in front of the robot
        auto ball_to_robot_angle = robot_ori_at_time.minDiff(
            (ball_pos_at_time - robot_pos_at_time).orientation());
        return ball_to_robot_angle < Angle::ofDegrees(45.0);
    }
};