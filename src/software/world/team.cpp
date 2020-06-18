#include "software/world/team.h"

#include <set>

#include "shared/constants.h"
#include "software/logger/logger.h"

Team::Team(const Duration& robot_expiry_buffer_duration, unsigned int buffer_size)
    : team_robots(),
      goalie_id(),
      robot_expiry_buffer_duration(robot_expiry_buffer_duration)
{
    // Set the size of the Timestamp history buffer
    last_update_timestamps.set_capacity(buffer_size);
    updateTimestamp(getMostRecentTimestampFromRobots());
}

Team::Team(const std::vector<Robot>& team_robots,
           const Duration& robot_expiry_buffer_duration)
    : Team(robot_expiry_buffer_duration)
{
    updateRobots(team_robots);
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

        auto it = std::find_if(team_robots.begin(), team_robots.end(),
                               [robot](const Robot& r) { return r.id() == robot.id(); });
        if (it != team_robots.end())
        {
            // The robot already exists on the team. Find and update the robot
            it->updateState(robot.currentState());
        }
        else
        {
            // This robot does not exist as part of the team yet. Add the new robot
            team_robots.emplace_back(robot);
        }
    }

    updateTimestamp(getMostRecentTimestampFromRobots());
}

void Team::updateState(const Team& new_team_data)
{
    updateRobots(new_team_data.getAllRobots());
    this->goalie_id = new_team_data.goalie_id;

    updateTimestamp(getMostRecentTimestampFromRobots());
}

void Team::updateStateToPredictedState(const Timestamp& timestamp)
{
    // Update the state of all robots to their predicted state
    for (auto it = team_robots.begin(); it != team_robots.end(); it++)
    {
        it->updateStateToPredictedState(timestamp);
    }

    updateTimestamp(timestamp);
}

void Team::removeExpiredRobots(const Timestamp& timestamp)
{
    // Check to see if any Robots have "expired". If it more time than the expiry_buffer
    // has passed, then remove the robot from the team
    for (auto it = team_robots.begin(); it != team_robots.end();)
    {
        Duration time_diff = timestamp - it->lastUpdateTimestamp();
        if (time_diff.getSeconds() < 0)
        {
            LOG(WARNING) << "Warning: tried to remove a robot at a negative time";
            it++;
        }
        if (time_diff > robot_expiry_buffer_duration)
        {
            it = team_robots.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void Team::removeRobotWithId(unsigned int robot_id)
{
    auto it = std::find_if(team_robots.begin(), team_robots.end(),
                           [&](const Robot& r) { return r.id() == robot_id; });

    if (it != team_robots.end())
    {
        team_robots.erase(it);
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
        LOG(WARNING) << "Warning: Tried to assign the goalie (id " << new_goalie_id
                     << ") to a robot that is not a member of the team" << std::endl;
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

Duration Team::getRobotExpiryBufferDuration() const
{
    return robot_expiry_buffer_duration;
}

void Team::setRobotExpiryBuffer(const Duration& new_robot_expiry_buffer_duration)
{
    robot_expiry_buffer_duration = new_robot_expiry_buffer_duration;
}

std::optional<Robot> Team::getRobotById(const unsigned int id) const
{
    for (const Robot& robot : team_robots)
    {
        if (robot.id() == id)
        {
            return robot;
        }
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

std::optional<unsigned int> Team::getGoalieID() const
{
    return goalie_id;
}

const std::vector<Robot>& Team::getAllRobots() const
{
    return team_robots;
}

std::vector<Robot> Team::getAllRobotsExceptGoalie() const
{
    auto goalie_robot = goalie();
    std::vector<Robot> all_robots;
    for (auto it = team_robots.begin(); it != team_robots.end(); it++)
    {
        if (goalie_robot && *it == *goalie_robot)
        {
            continue;
        }
        all_robots.emplace_back(*it);
    }

    return all_robots;
}


void Team::clearAllRobots()
{
    team_robots.clear();
}

boost::circular_buffer<Timestamp> Team::getTimestampHistory() const
{
    return last_update_timestamps;
}

Timestamp Team::getMostRecentTimestamp() const
{
    return last_update_timestamps.front();
}

Timestamp Team::getMostRecentTimestampFromRobots()
{
    std::vector<Robot> robots = this->getAllRobots();

    Timestamp most_recent_timestamp = Timestamp::fromSeconds(0);

    for (Robot robot : robots)
    {
        if (robot.lastUpdateTimestamp() > most_recent_timestamp)
        {
            most_recent_timestamp = robot.lastUpdateTimestamp();
        }
    }

    return most_recent_timestamp;
}

void Team::updateTimestamp(Timestamp time_stamp)
{
    // Check if the timestamp buffer is empty
    if (last_update_timestamps.empty())
    {
        last_update_timestamps.push_front(time_stamp);
    }
    // Check that the new timestamp is not older than the most recent timestamp
    else if (time_stamp < Team::getMostRecentTimestamp())
    {
        throw std::invalid_argument(
            "Error: Attempt tp update Team state with old Timestamp");
    }
    else if (time_stamp == Team::getMostRecentTimestamp())
    {
        // Don't update if the timestamp is the same as the most recent already assigned
        // to Team
        return;
    }
    else
    {
        last_update_timestamps.push_front(time_stamp);
    }
}

std::optional<Timestamp> Team::lastUpdateTimestamp() const
{
    std::optional<Timestamp> most_recent_timestamp = std::nullopt;
    for (const Robot& robot : getAllRobots())
    {
        if (!most_recent_timestamp || robot.lastUpdateTimestamp() > most_recent_timestamp)
        {
            most_recent_timestamp = robot.lastUpdateTimestamp();
        }
    }
    return most_recent_timestamp;
}

bool Team::operator==(const Team& other) const
{
    return this->getAllRobots() == other.getAllRobots() &&
           this->goalie_id == other.goalie_id &&
           this->robot_expiry_buffer_duration == other.robot_expiry_buffer_duration;
}

bool Team::operator!=(const Team& other) const
{
    return !(*this == other);
}
