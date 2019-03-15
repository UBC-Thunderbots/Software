#include "ai/hl/stp/evaluation/team.h"

std::optional<Robot> Evaluation::nearest_robot(const Team team, const Point ref_point)
{
    if (team.getAllRobots().empty())
    {
        return std::nullopt;
    }

    unsigned nearestRobotId = team.getAllRobots()[0].id();
    double minDist          = (ref_point - team.getAllRobots()[0].position()).len();

    for (const Robot &curRobot : team.getAllRobots())
    {
        double curDistance = (ref_point - curRobot.position()).len();
        if (curDistance < minDist)
        {
            nearestRobotId = curRobot.id();
            minDist        = curDistance;
        }
    }

    return team.getRobotById(nearestRobotId).value();
}
