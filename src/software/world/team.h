#pragma once

#include <cstdlib>
#include <map>
#include <optional>
#include <vector>

#include "software/time/timestamp.h"
#include "software/world/robot.h"

/**
 * A team of robots
 */
class Team
{
   public:
    /**
     * Create a new team
     *
     * @param robot_expiry_buffer_duration The Duration for which a robot must not
     * have been updated for before it is removed from the team
     */
    explicit Team(
        const Duration& robot_expiry_buffer_duration = Duration::fromMilliseconds(50));

    /**
     * Create a new team
     *
     * @param team_robots The robots on the team
     * @param robot_expiry_buffer_duration The Duration for which a robot must not
     * have been updated for before it is removed from the team
     */
    explicit Team(
        const std::vector<Robot>& team_robots,
        const Duration& robot_expiry_buffer_duration = Duration::fromMilliseconds(50));

    /**
     * Creates a new team based on the TbotsProto::Team protobuf representation
     *
     * @param team_proto The TbotsProto::Team protobuf which this robot should be based on
     */
    explicit Team(
        const TbotsProto::Team& team_proto,
        const Duration& robot_expiry_buffer_duration = Duration::fromMilliseconds(50));

    /**
     * Updates this team with new robots.
     *
     * @throws std::invalid_argument if multiple robots have the same id
     * @param team_robots the new robots for this team
     */
    void updateRobots(const std::vector<Robot>& team_robots);

    /**
     * Updates this team with new data from the given team object. This is different from
     * a copy constructor because the team object is only used to store data, we don't
     * take the entire state of the new_team_data. For example, the robots on this team
     * may have complex internal state for predicting movement. The "simple" robot data
     * such as position, velocity... from the new_team_data is used to update the state of
     * the robots on this team, rather than the robots simply being copied over
     * (because if we copied we would lose our state).
     *
     * @param new_team_data A team with the new team data
     */
    void updateState(const Team& new_team_data);

    /**
     * Removes expired robots from the team. Robots are expired if it has been more than
     * the expiry_buffer time has passed since they were last updated. This would happen
     * if a robot is removed from the field, so that it is no longer seen by the cameras.
     * After a short amount of time, we should treat robots that we can no longer see
     * (and therefore have not been updating) as removed from the field, so we should
     * remove them from the team.
     *
     * @param timestamp The timestamp for when this removal is taking place
     */
    void removeExpiredRobots(const Timestamp& timestamp);

    /**
     * Remove the robot with the given ID from the team
     *
     * If there is no robot with the given id on the team, does nothing
     *
     * @param robot_id The id of the robot to remove
     */
    void removeRobotWithId(unsigned int robot_id);

    /**
     * Assigns the goalie id for this team
     *
     * @param new_goalie_id The id of the new goalie for this team
     *
     */
    void assignGoalie(RobotId new_goalie_id);

    /**
     * Clears the goalie for this team. There will be no goalie assigned after
     * this operation
     *
     */
    void clearGoalie();

    /**
     * Gets the number of robots on this team
     *
     * @return the number of robots on this team
     */
    size_t numRobots() const;

    /**
     * Returns the Duration for which a Robot must not have been updated for before
     * being removed from this team.
     *
     * @return the Duration for which a Robot must not have been updated for before
     * being removed from this team.
     */
    const Duration& getRobotExpiryBufferDuration() const;

    /**
     * Sets the Duration for which a Robot must not have been updated for before being
     * removed from this team.
     *
     * @param new_robot_expiry_buffer_duration the Duration for which a Robot must
     * not have been updated for before being removed from this team.
     */
    void setRobotExpiryBuffer(const Duration& new_robot_expiry_buffer_duration);

    /**
     * Updates the unavailable capabilities of a specific robot. If the team does
     * not have that robot, does not do anything.
     *
     * @param id the id of the desired Robot
     * @param new_unavailable_robot_capabilities a set of the updated unavailable
     * robot capabilities
     * */
    void setUnavailableRobotCapabilities(
        RobotId id, const std::set<RobotCapability>& new_unavailable_robot_capabilities);

    /**
     * Returns the robot with the given id. If this team does not have that robot,
     * returns an std::nullopt
     *
     * @param id the id of the desired Robot
     *
     * @return the Robot on the team with the given id if it exists, otherwise
     * std::nullopt
     */
    std::optional<Robot> getRobotById(const unsigned int id) const;

    /**
     * Returns the goalie robot for this team, if one is specified. Otherwise
     * returns std::nullopt
     *
     * @return the goalie robot for this team if one is specified, otherwise
     * returns std::nullopt
     */
    std::optional<Robot> goalie() const;

    /**
     * Returns the ID of the goalie for this team, if one has been specified. Otherwise
     * returns std::nullopt
     *
     * @return The ID of the goalie robot for this team if one is specified, otherwise
     * returns std::nullopt
     */
    std::optional<unsigned int> getGoalieId() const;

    /**
     * Returns a vector of all the robots on this team.
     *
     * @return a vector of all the robots on this team.
     */
    const std::vector<Robot>& getAllRobots() const;

    /**
     * Returns a vector of all the robots on this team excluding the goalie
     *
     * @return a vector of all the robots on this team excluding the goalie
     */
    std::vector<Robot> getAllRobotsExceptGoalie() const;

    /**
     * Finds the robot on a team that is closest to the reference point
     *
     * @param ref_point The point where the distance to each robot will be measured.
     * @return Robot that is closest to the reference point.
     *         std::nullopt otherwise
     */
    std::optional<Robot> getNearestRobot(const Point& ref_point) const;

    /**
     * Given a list of robots, finds the robot on that team that is closest to a
     * reference point.
     *
     * @param robots the list of robots
     * @param ref_point The point where the distance to each robot will be measured.
     * @return Robot that is closest to the reference point.
     */
    static std::optional<Robot> getNearestRobot(const std::vector<Robot>& robots,
                                                const Point& ref_point);

    /**
     * Removes all Robots from this team. Does not affect the goalie id.
     */
    void clearAllRobots();

    /**
     * Returns the timestamp of the most recently updated robot on this team
     *
     * @return the timestamp of the most recently updated robot on this team, or
     *         std::nullopt if this team is empty
     */
    std::optional<Timestamp> timestamp() const;

    /**
     * Defines the equality operator for a Team. Teams are equal if their robots are equal
     * and have the same robot assigned as the goalie
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

    /**
     * Returns the most Timestamp corresponding to the most recent update to Field object
     *
     * @return Timestamp : The Timestamp corresponding to the most recent update to the
     * Field object
     */
    Timestamp getMostRecentTimestamp() const;

    /**
     * Set robots in the team to be injured
     *
     * @param robot_ids robot ids of injured robots
     */

    void setInjuredRobots(const std::vector<Robot>& robots) const;

    /**
     * Returns a list of robot ids for robots that are injured
     *
     * @return a list of injured robots
     */
    std::vector<Robot> getInjuredRobots() const;


   private:
    /**
     * Updates the last update timestamp
     *
     * @param timestamp The last time this Team was updated
     *
     * @throws std::invalid_argument if timestamp is older than the current last update
     * timestamp
     */
    void updateTimestamp(Timestamp timestamp);

    /**
     * Returns the most recent Timestamp from all robots in a Team
     *
     * @return The most revent Timestamp from all robots in the Team
     */
    Timestamp getMostRecentTimestampFromRobots();

    // The robots on this team
    std::vector<Robot> team_robots_;

    // The robot id of the goalie for this team
    std::optional<unsigned int> goalie_id_;

    // The duration for which a Robot must not have been updated for before
    // being removed from this team.
    Duration robot_expiry_buffer_duration_;

    Timestamp last_update_timestamp_;

    // the robots in the team that are injured
    mutable std::vector<Robot> injured_robots;
};

enum class TeamType
{
    FRIENDLY,
    ENEMY,
};
