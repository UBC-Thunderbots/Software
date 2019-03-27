#include <shared/constants.h>
#include "ai/hl/stp/evaluation/robot.h"



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

bool Evaluation::robotHasPossession(const Ball ball, const Robot robot)
{
    // The actual vector to the ball from the center of the robot
    Vector robot_center_to_ball = ball.position() - robot.position();

    // Calculate the ideal vector from the robot to the ball for the robot to have possession.
    Angle orientation = robot.orientation();
    double expected_position_x = orientation.cos() * DIST_TO_FRONT_OF_ROBOT_METERS;
    double expected_position_y = orientation.sin() * DIST_TO_FRONT_OF_ROBOT_METERS;
    Vector expected_point = Vector(expected_position_x, expected_position_y);

    return robot_center_to_ball.isClose(expected_point, DRIBBLER_LENGTH/2);
};
