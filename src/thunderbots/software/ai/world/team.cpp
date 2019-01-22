#include "ai/world/team.h"

#include <set>

#include "shared/constants.h"

Team::Team(const std::chrono::milliseconds robot_expiry_buffer_milliseconds)
    : team_robots(),
      goalie_id(),
      robot_expiry_buffer_milliseconds(robot_expiry_buffer_milliseconds)
{
}

void Team::updateRobots(const std::vector<Robot>& new_robots)
{
    // Update the robots, checking that there are no duplicate IDs in the given data
    std::set<unsigned int> robot_ids;
    for (const auto& robot : new_robots)
    {
        // The second value of the pair that is returned indicates whether or not the
        // value was already present in the set. We use this to detect duplicate robots
        auto duplicate_id = robot_ids.insert(robot.id());
        if (!duplicate_id.second)
        {
            throw std::invalid_argument(
                "Error: Multiple robots on the same team with the same id");
        }

        auto it = team_robots.find(robot.id());
        if (it != team_robots.end())
        {
            // The robot already exists on the team. Find and update the robot
            team_robots.at(robot.id()).updateState(robot);
        }
        else
        {
            // This robot does not exist as part of the team yet. Add the new robot
            team_robots.insert(std::make_pair(robot.id(), robot));
        }
    }
}

void Team::updateState(const Team& new_team_data)
{
    updateRobots(new_team_data.getAllRobots());
    this->goalie_id = new_team_data.goalie_id;
}

void Team::updateStateToPredictedState(
    const std::chrono::steady_clock::time_point timestamp)
{
    // Update the state of all robots to their predicted state
    for (auto it = team_robots.begin(); it != team_robots.end(); it++)
    {
        it->second.updateStateToPredictedState(timestamp);
    }
}

void Team::removeExpiredRobots(const std::chrono::steady_clock::time_point timestamp)
{
    // Check to see if any Robots have "expired". If it more time than the expiry_buffer
    // has passed, then remove the robot from the team
    for (auto it = team_robots.begin(); it != team_robots.end();)
    {
        if (std::chrono::duration(timestamp - it->second.lastUpdateTimestamp()) >
            robot_expiry_buffer_milliseconds)
        {
            it = team_robots.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void Team::assignGoalie(unsigned int new_goalie_id)
{
    if (getRobotById(new_goalie_id))
    {
        goalie_id = new_goalie_id;
    }
    else
    {
        throw std::invalid_argument(
            "Error: Assigning the goalie to a robot that is not a member of the team");
    }
}

void Team::clearGoalie()
{
    goalie_id.reset();
}

std::size_t Team::numRobots() const
{
    return team_robots.size();
}

std::chrono::milliseconds Team::getRobotExpiryBufferMilliseconds()
{
    return robot_expiry_buffer_milliseconds;
}

void Team::setRobotExpiryBuffer(
    std::chrono::milliseconds new_robot_expiry_buffer_milliseconds)
{
    robot_expiry_buffer_milliseconds = new_robot_expiry_buffer_milliseconds;
}

std::optional<Robot> Team::getRobotById(const unsigned int id) const
{
    auto it = team_robots.find(id);
    if (it != team_robots.end())
    {
        return it->second;
    }

    return std::nullopt;
}

std::optional<Robot> Team::goalie() const
{
    if (goalie_id)
    {
        return getRobotById(*goalie_id);
    }

    return std::nullopt;
}

std::vector<Robot> Team::getAllRobots() const
{
    std::vector<Robot> all_robots;
    for (auto it = team_robots.begin(); it != team_robots.end(); it++)
    {
        all_robots.emplace_back(it->second);
    }

    return all_robots;
}

void Team::clearAllRobots()
{
    team_robots.clear();
}

bool Team::operator==(const Team& other) const
{
    return this->getAllRobots() == other.getAllRobots() &&
           this->goalie_id == other.goalie_id &&
           this->robot_expiry_buffer_milliseconds ==
               other.robot_expiry_buffer_milliseconds;
}

bool Team::operator!=(const Team& other) const
{
    return !(*this == other);
}
