#include "software/ai/evaluation/team.h"


std::optional<Robot> nearestRobot(const Team &team, const Point &ref_point)
{
    return nearestRobot(team.getAllRobots(), ref_point);
}

std::optional<Robot> nearestRobot(const std::vector<Robot> &robots,
                                  const Point &ref_point)
{
    if (robots.empty())
    {
        return std::nullopt;
    }

    Robot nearest_robot = robots.at(0);
    for (const Robot &curRobot : robots)
    {
        double curDistance = (ref_point - curRobot.position()).length();
        if (curDistance < (nearest_robot.position() - ref_point).length())
        {
            nearest_robot = curRobot;
        }
    }

    return nearest_robot;
}
