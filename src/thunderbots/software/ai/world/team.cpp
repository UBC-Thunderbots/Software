#include "team.h"
#include <shared/constants.h>
#include "util/parameter/dynamic_parameters.h"

// We initialize the goalie_id to -1, a value that will never occur naturally as all robot
// ids are numbers >= 0 because otherwise the goalie_id will be initialized to 0 by
// default, and we want there to be no goalie until the value is updated.
Team::Team() : team_robots(), goalie_id(-1)
{
}

void Team::update(const thunderbots_msgs::Team& team_msg)
{
    std::vector<Robot> new_robots;
    for (const auto& robot : team_msg.robots)
    {
        new_robots.emplace_back(Robot(robot));
    }

    updateRobots(new_robots);
}

void Team::update(const Team& new_team_data)
{
    updateRobots(new_team_data.getAllRobots());
    this->goalie_id = new_team_data.goalie_id;
}

void Team::updateRobots(const std::vector<Robot>& new_robots)
{
    AITimestamp timestamp_now = Timestamp::getTimestampNow();

    // Update the robots, checking that there are no duplicate IDs in the given data
    std::set<unsigned int> robot_ids;
    for (const auto& robot : new_robots)
    {
        auto no_duplicate = robot_ids.insert(robot.id());
        if (!no_duplicate.second)
        {
            // TODO: Multiple robots on the same team with the same id. Throw exception
            // See https://github.com/UBC-Thunderbots/Software/issues/16
        }

        auto it = team_robots.find(robot.id());
        if (it != team_robots.end())
        {
            team_robots.at(robot.id())
                .first.update(robot.position(), robot.velocity(), robot.orientation(),
                              robot.angularVelocity());
            team_robots.at(robot.id()).second = timestamp_now;
        }
        else
        {
            team_robots.insert(
                std::make_pair(robot.id(), std::make_pair(robot, timestamp_now)));
        }
    }

    // Check to see if any Robots have "expired". If it has been a while since they have
    // been updated, then they have likely been removed from the world and should be
    // removed from the team.
    for (auto it = team_robots.begin(); it != team_robots.end();)
    {
        unsigned int milliseconds_since_last_update = static_cast<unsigned int>(
            Timestamp::getMicroseconds(timestamp_now - it->second.second) *
            MILLISECONDS_PER_MICROSECOND);
        if (milliseconds_since_last_update >
            DynamicParameters::robot_vision_debounce_milliseconds.value())
        {
            team_robots.erase(it++);
        }
        else
        {
            it++;
        }
    }
}

void Team::updateGoalie(unsigned int new_goalie_id)
{
    goalie_id = new_goalie_id;
}

std::size_t Team::size() const
{
    return team_robots.size();
}

std::optional<Robot> Team::getRobotById(unsigned int id) const
{
    auto it = team_robots.find(id);
    if (it != team_robots.end())
    {
        return it->second.first;
    }

    return std::nullopt;
}

std::optional<Robot> Team::goalie() const
{
    return getRobotById(goalie_id);
}

std::vector<Robot> Team::getAllRobots() const
{
    std::vector<Robot> all_robots;
    for (auto it = team_robots.begin(); it != team_robots.end(); it++)
    {
        all_robots.emplace_back(it->second.first);
    }

    return all_robots;
}

void Team::clearAllRobots()
{
    team_robots.clear();
}

bool Team::operator==(const Team& other) const
{
    // The order of keys is deterministic when iterating over a map (how we create the
    // allRobots vectors), so if the teams have the same robots, the vectors will contain
    // the same robots in the same order and be equal
    bool robots_eq = this->getAllRobots() == other.getAllRobots();
    bool goalie_eq = this->goalie_id == other.goalie_id;
    return robots_eq && goalie_eq;
}

bool Team::operator!=(const Team& other) const
{
    return !(*this == other);
}