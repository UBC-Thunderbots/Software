#include "software/ai/hl/stp/evaluation/robot.h"

#include "shared/constants.h"


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
    int ball_index  = 0;
    int robot_index = 0;
    if (timestamp.getMilliseconds() != 0)
    {
        // Get the indices for the state information at the desired timestamp.
        auto ball_index_result  = ball.getHistoryIndexFromTimestamp(timestamp);
        auto robot_index_result = robot.getHistoryIndexFromTimestamp(timestamp);
        if (!ball_index_result || !robot_index_result)
            return false;

        ball_index  = ball_index_result.value();
        robot_index = robot_index_result.value();
    }

    // The actual vector to the ball from the center of the robot
    Vector robot_center_to_ball = ball.getPreviousStates()[ball_index].position() -
                                  robot.getPreviousPositions()[robot_index];

    // Calculate the ideal vector from the robot to the ball for the robot to have
    // possession.
    Angle orientation = robot.getPreviousOrientations()[robot_index];
    Vector expected_point =
        Point::createFromAngle(orientation).norm(DIST_TO_FRONT_OF_ROBOT_METERS);

    return robot_center_to_ball.isClose(expected_point, DRIBBLER_WIDTH / 2);
};
