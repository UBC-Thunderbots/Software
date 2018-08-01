#include "team.h"

Team::Team()
{
}

void Team::update(std::vector<Robot> &team_robots)
{

}

void Team::update(const thunderbots_msgs::Team &team_msg) {

}

std::size_t Team::size() const
{
    return static_cast<size_t>(0);
}

std::optional<Robot> Team::getRobotById(unsigned int id) const
{
    return std::nullopt;
}

std::optional<Robot> Team::goalie() const
{
    return std::nullopt;
}

std::vector<Robot> Team::getAllRobots() const
{
    return std::vector<Robot>();
}