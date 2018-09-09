#pragma once

#include <cstdlib>
#include <map>
#include <optional>
#include <vector>
#include "ai/world/robot.h"
#include "thunderbots_msgs/Team.h"
#include "util/timestamp.h"

/**
 * Defines the available colors for an SSL team
 */
typedef enum { BLUE = 0, YELLOW = 1 } TeamColour;

/**
 * A team of robots
 */
class Team
{
   public:
    /**
     * Create a new team
     */
    explicit Team();

    /**
     * Updates this team with new data from the team message
     *
     * @param team_msg the message with the new team data
     */
    void update(const thunderbots_msgs::Team& team_msg);

    /**
     * Updates this team with new data from the given team object
     *
     * @param new_team_data A team with the new team data
     */
    void update(const Team& new_team_data);

    /**
     * Updates this team with new robots.
     *
     * @param new_robots the new robots for this team
     */
    void updateRobots(const std::vector<Robot>& new_robots);

    /**
     * Updates the goalie for this team, making it the robot with the newly given id
     *
     * @param new_goalie_id The id of the new goalie for this team
     */
    void updateGoalie(unsigned int new_goalie_id);

    /**
     * Gets the number of robots on this team
     *
     * @return the number of robots on this team
     */
    std::size_t size() const;

    /**
     * Returns the robot with the given id. If this team does not have that robot,
     * returns an std::nullopt
     *
     * @param id the id of the desired Robot
     *
     * @return the Robot on the team with the given id if it exists, otherwise
     * std::nullopt
     */
    std::optional<Robot> getRobotById(unsigned int id) const;

    /**
     * Returns the goalie robot for this team, if one is specified. Otherwise
     * returns std::nullopt
     *
     * @return the goalie robot for this team if one is specified, otherwise
     * returns std::nullopt
     */
    std::optional<Robot> goalie() const;

    /**
     * Returns a vector of all the robots on this team.
     *
     * @return a vector of all the robots on this team.
     */
    std::vector<Robot> getAllRobots() const;

    /**
     * Removes all Robots from this team. Does not affect the goalie id.
     */
    void clearAllRobots();

    /**
     * Defines the equality operator for a Team. Teams are equal if their robots and
     * goalies are the same. The timestamps for when the robots were updated do not need
     * to be the same for teams to be considered equal.
     *
     * @param other The team to compare against for equality
     * @return True if the other team is equal to this team, and false otherwise
     */
    bool operator==(const Team& other) const;

    /**
     * Defines the inequality operator for a Team.
     *
     * @param other The team to compare against for inequality
     * @return True if the other team is not equal to this team, and false otherwise
     */
    bool operator!=(const Team& other) const;

   private:
    // The map that contains the Robots for this team. The map makes it easier to help
    // guarantee we only have robots with unique IDs.
    //
    // The pair contains both a Robot and the timestamp for when the Robot was last seen
    // on vision and updated. This helps provide a buffer for when a robot briefly
    // disappears from the overhead cameras, so that we aren't constantly adding and
    // removing a robot from the team if it is flickering in vision.
    std::map<unsigned int, std::pair<Robot, AITimestamp>> team_robots;

    // The robot id of the goalie for this team
    int goalie_id;
};
