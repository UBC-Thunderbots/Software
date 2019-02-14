#include "ai/hl/stp/evaluation/team.h"


using namespace Evaluation;

Robot nearest_friendly(const Team &friendly_team, const Point &ref_point)
{

    unsigned nearestRobotId = friendly_team.getAllRobots()[0].id();
    double minDist = (ref_point - friendly_team.getAllRobots()[0].position()).len();

    for (const Robot &curRobot : friendly_team.getAllRobots())
    {
        double curDistance = (ref_point - curRobot.position()).len();
        if (!nearestRobotId || curDistance < minDist)
        {
            nearestRobotId = curRobot.id();
            minDist = curDistance;
        }
    }

    return friendly_team.getRobotById(nearestRobotId).value();
}
