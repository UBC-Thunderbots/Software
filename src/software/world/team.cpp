#include "software/world/team.h"

#include <set>

#include "shared/constants.h"
#include "software/logger/logger.h"

Team::Team(const Duration& robot_expiry_buffer_duration)
    : team_robots(),
      goalie_id(),
      robot_expiry_buffer_duration(robot_expiry_buffer_duration),
      last_update_timestamp()
{
    updateTimestamp(getMostRecentTimestampFromRobots());
}

Team::Team(const std::vector<Robot>& team_robots,
           const Duration& robot_expiry_buffer_duration)
    : Team(robot_expiry_buffer_duration)
{
    updateRobots(team_robots);
}

Team::Team(const TbotsProto::Team& team_proto,
           const Duration& robot_expiry_buffer_duration)
    : robot_expiry_buffer_duration(robot_expiry_buffer_duration), last_update_timestamp()
{
    if (team_proto.has_goalie_id())
    {
        goalie_id = team_proto.goalie_id();
    }
    else
    {
        goalie_id = std::nullopt;
    }

    for (int i = 0; i < team_proto.team_robots_size(); i++)
    {
        team_robots.emplace_back(Robot(team_proto.team_robots(i)));
    }
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
            it->updateState(robot.currentState(), robot.timestamp());
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

void Team::removeExpiredRobots(const Timestamp& timestamp)
{
    // Check to see if any Robots have "expired". If it more time than the expiry_buffer
    // has passed, then remove the robot from the team
    for (auto it = team_robots.begin(); it != team_robots.end();)
    {
        Duration time_diff = timestamp - it->timestamp();
        if (time_diff.toSeconds() < 0)
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
    goalie_id = new_goalie_id;
}

void Team::clearGoalie()
{
    goalie_id.reset();
}

std::size_t Team::numRobots() const
{
    return team_robots.size();
}

const Duration& Team::getRobotExpiryBufferDuration() const
{
    return robot_expiry_buffer_duration;
}

void Team::setRobotExpiryBuffer(const Duration& new_robot_expiry_buffer_duration)
{
    robot_expiry_buffer_duration = new_robot_expiry_buffer_duration;
}

void Team::setUnavailableRobotCapabilities(
    RobotId id, const std::set<RobotCapability>& new_unavailable_robot_capabilities)
{
    for (Robot& robot : team_robots)
    {
        if (robot.id() == id)
        {
            robot.getMutableRobotCapabilities() = new_unavailable_robot_capabilities;
            return;
        }
    }
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

std::optional<unsigned int> Team::getGoalieId() const
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

std::optional<Robot> Team::getNearestRobot(const Point& ref_point) const
{
    return getNearestRobot(this->getAllRobots(), ref_point);
}

std::optional<Robot> Team::getNearestRobot(const std::vector<Robot>& robots,
                                           const Point& ref_point)
{
    if (robots.empty())
    {
        return std::nullopt;
    }

    Robot nearest_robot = robots.at(0);
    for (const Robot& curRobot : robots)
    {
        double curDistance = (ref_point - curRobot.position()).length();
        if (curDistance < (nearest_robot.position() - ref_point).length())
        {
            nearest_robot = curRobot;
        }
    }

    return nearest_robot;
}

void Team::clearAllRobots()
{
    team_robots.clear();
}

Timestamp Team::getMostRecentTimestamp() const
{
    return last_update_timestamp;
}

Timestamp Team::getMostRecentTimestampFromRobots()
{
    std::vector<Robot> robots = this->getAllRobots();

    Timestamp most_recent_timestamp = Timestamp::fromSeconds(0);

    for (Robot robot : robots)
    {
        if (robot.timestamp() > most_recent_timestamp)
        {
            most_recent_timestamp = robot.timestamp();
        }
    }

    return most_recent_timestamp;
}

void Team::updateTimestamp(Timestamp timestamp)
{
    // Check that the new timestamp is not older than the most recent timestamp
    if (timestamp < Team::getMostRecentTimestamp())
    {
        throw std::invalid_argument(
            "Error: Attempt tp update Team state with old Timestamp");
    }
    else
    {
        last_update_timestamp = timestamp;
    }
}

std::optional<Timestamp> Team::timestamp() const
{
    std::optional<Timestamp> most_recent_timestamp = std::nullopt;
    for (const Robot& robot : getAllRobots())
    {
        if (!most_recent_timestamp || robot.timestamp() > most_recent_timestamp)
        {
            most_recent_timestamp = robot.timestamp();
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
