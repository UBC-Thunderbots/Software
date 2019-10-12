#include "software/ai/evaluation/team.h"


std::optional<Robot> Evaluation::nearestRobot(const Team &team, const Point &ref_point)
{
    return nearestRobot(team.getAllRobots(), ref_point);
}

std::optional<Robot> Evaluation::nearestRobot(const std::vector<Robot> &robots,
                                              const Point &ref_point)
{
    if (robots.empty())
    {
        return std::nullopt;
    }

    Robot nearest_robot = robots.at(0);
    for (const Robot &curRobot : robots)
    {
        double curDistance = (ref_point - curRobot.position()).len();
        if (curDistance < (nearest_robot.position() - ref_point).len())
        {
            nearest_robot = curRobot;
        }
    }

    return nearest_robot;
}
