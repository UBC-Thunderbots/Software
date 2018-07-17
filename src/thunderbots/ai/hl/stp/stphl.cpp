#include "stphl.h"

STPHL::STPHL()
{
}

std::vector<std::pair<unsigned int, Intent>> STPHL::getIntentAssignment(
    const World &world)
{
    std::vector<std::pair<unsigned int, Intent>> result =
        std::vector<std::pair<unsigned int, Intent>>();

    std::vector<Robot> friendly_robots = world.friendlyTeam().getAllRobots();
    for (Robot robot : friendly_robots)
    {
        result.emplace_back(std::make_pair(robot.id(), Intent()));
    }

    return result;
}
