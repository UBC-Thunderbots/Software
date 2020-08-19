#include "software/sensor_fusion/filter/possession_filter.h"

std::vector<RobotIdWithTeamSide> getRobotsWithPossession(
    std::vector<RobotId> friendly_robots_with_breakbeam_triggered, Team friendly_team,
    Team enemy_team, Ball ball)
{
    std::vector<RobotIdWithTeamSide> possessions;
    for (const auto &robot_id : friendly_robots_with_breakbeam_triggered)
    {
        possessions.push_back(
            RobotIdWithTeamSide{.id = robot_id, .team_side = TeamSide::FRIENDLY});
    }

    for (const auto &robot : friendly_team.getAllRobots())
    {
        if (ballNearDribbler(ball.position(), robot.position(), robot.orientation()))
        {
            possessions.push_back(
                RobotIdWithTeamSide{.id = robot.id(), .team_side = TeamSide::FRIENDLY});
        }
    }

    for (const auto &robot : enemy_team.getAllRobots())
    {
        if (ballNearDribbler(ball.position(), robot.position(), robot.orientation()))
        {
            possessions.push_back(
                RobotIdWithTeamSide{.id = robot.id(), .team_side = TeamSide::ENEMY});
        }
    }

    return possessions;
}

std::optional<RobotId> getRobotWithPossession(
    const Ball &ball, const Team &team,
    const std::vector<RobotId> &robots_with_breakbeam_triggered,
    double possession_distance_threshold)
{
    return std::nullopt;
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

bool ballNearDribbler(Point ball_position, Point robot_position, Angle robot_orientation)
{
    static const double POSSESSION_THRESHOLD_METERS = ROBOT_MAX_RADIUS_METERS + 0.2;
    if ((ball_position - robot_position).length() > POSSESSION_THRESHOLD_METERS)
    {
        return false;
    }
    else
    {
        // check that ball is in a 90-degree cone in front of the robot
        auto ball_to_robot_angle =
            robot_orientation.minDiff((ball_position - robot_position).orientation());
        return (ball_to_robot_angle < Angle::fromDegrees(45.0));
    }
}
