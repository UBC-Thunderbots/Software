#pragma once

#include <cstdlib>
#include <map>
#include <optional>
#include <vector>
#include "ai/world/robot.h"

/**
 * Defines the available colors for an SSL team
 */
typedef enum { BLUE = 0, YELLOW = 1 } TEAM_COLOUR;

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
     * Updates this team with new robots.
     *
     * @param team_robots the new robots for this team
     */
    void update(std::vector<Robot> &team_robots);

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

   private:
    std::map<unsigned int, Robot> team_;
};
