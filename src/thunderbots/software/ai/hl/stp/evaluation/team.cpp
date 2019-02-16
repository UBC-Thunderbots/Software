#include "ai/hl/stp/evaluation/team.h"


std::optional<Robot> Evaluation::nearest_friendly(const Team friendly_team,
                                                  const Point ref_point)
{
    if (friendly_team.getAllRobots().empty())
    {
        return std::nullopt;
    }

    unsigned nearestRobotId = friendly_team.getAllRobots()[0].id();
    double minDist = (ref_point - friendly_team.getAllRobots()[0].position()).len();

    for (const Robot &curRobot : friendly_team.getAllRobots())
    {
        double curDistance = (ref_point - curRobot.position()).len();
        if (curDistance < minDist)
        {
            nearestRobotId = curRobot.id();
            minDist        = curDistance;
        }
    }

    return friendly_team.getRobotById(nearestRobotId).value();
}
