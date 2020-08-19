#include "software/sensor_fusion/filter/possession_filter.h"

std::optional<RobotId> getRobotWithPossession(
    const Ball &ball, const Team &team,
    const std::vector<RobotId> &robots_with_breakbeam_triggered,
    double possession_distance_threshold)
{
    if (robots_with_breakbeam_triggered.size() == 1)
    {
        auto breakbeam_robot = team.getRobotById(robots_with_breakbeam_triggered[0]);
        if (breakbeam_robot &&
            getPossessionDistance(ball.position(), breakbeam_robot->position(),
                                  breakbeam_robot->orientation(),
                                  possession_distance_threshold))
        {
            return breakbeam_robot->id();
        }
    }

    std::optional<RobotId> robot_with_possession;
    double min_possession_distance = possession_distance_threshold;
    for (const auto &robot : team.getAllRobots())
    {
        auto possession_distance =
            getPossessionDistance(ball.position(), robot.position(), robot.orientation(),
                                  possession_distance_threshold);
        if (possession_distance && possession_distance < min_possession_distance)
        {
            min_possession_distance = *possession_distance;
            robot_with_possession   = robot.id();
        }
    }
    return robot_with_possession;
}

std::optional<double> getPossessionDistance(Point ball_position, Point robot_position,
                                            Angle robot_orientation,
                                            double possession_distance_threshold)
{
    double ball_to_robot_distance = (ball_position - robot_position).length();
    if (ball_to_robot_distance <= possession_distance_threshold)
    {
        // check that ball is in a 90-degree cone in front of the robot
        auto ball_to_robot_angle =
            robot_orientation.minDiff((ball_position - robot_position).orientation());
        if (ball_to_robot_angle < Angle::fromDegrees(45.0))
        {
            return ball_to_robot_distance;
        }
    }
    return std::nullopt;
}
