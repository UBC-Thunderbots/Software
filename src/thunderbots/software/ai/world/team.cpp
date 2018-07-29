#include "team.h"

Team::Team()
{
    team_ = std::map<unsigned int, Robot>();
}

void Team::update(std::vector<Robot> &team_robots)
{
    team_.clear();
    for (Robot robot : team_robots)
    {
        team_.insert(std::pair<unsigned int, Robot>(robot.id(), robot));
    }
}

std::size_t Team::size() const
{
    return team_.size();
}

std::optional<Robot> Team::getRobotById(unsigned int id) const
{
    if (team_.count(id) == 0)
    {
        return std::nullopt;
    }

    return team_.at(id);
}

std::optional<Robot> Team::goalie() const
{
    if (!team_.empty())
    {
        return team_.at(0);
    }

    return std::nullopt;
}

std::vector<Robot> Team::getAllRobots() const
{
    std::vector<Robot> all_robots = std::vector<Robot>();
    for (auto it = team_.begin(); it != team_.end(); it++)
    {
        all_robots.emplace_back(it->second);
    }

    return all_robots;
}