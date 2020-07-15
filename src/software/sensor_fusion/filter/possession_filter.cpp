#include "software/sensor_fusion/filter/possession_filter.h"

PossessionFilter::PossessionFilter(double possession_threshold_meters)
    : possession_threshold_meters(possession_threshold_meters)
{
}

std::vector<RobotIdWithTeamSide> PossessionFilter::getRobotsWithPossession(
    std::vector<RobotId> friendly_robots_with_breakbeam_triggered, Team friendly_team,
    Team enemy_team, Ball ball) const
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

bool PossessionFilter::ballNearDribbler(Point ball_position, Point robot_position,
                                        Angle robot_orientation) const
{
    if ((ball_position - robot_position).length() > possession_threshold_meters)
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
