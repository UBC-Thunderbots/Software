#pragma once

#include <map>
#include <optional>

#include "software/time/timestamp.h"
#include "software/world/robot_state.h"
#include "software/world/team_types.h"

/**
 * A light structure for a robot ID with TeamSide
 */
struct RobotIdWithTeamSide
{
    unsigned int id;
    TeamSide team_side;

    bool operator==(const RobotIdWithTeamSide &other) const
    {
        return id == other.id && team_side == other.team_side;
    }

    bool operator<(const RobotIdWithTeamSide &other) const
    {
        return id < other.id || team_side < other.team_side;
    }
};

class TimestampedPossessionState
{
   public:
    /**
     * Creates a new possession state that tracks no robots' possession and has timestamp
     * of 0
     */
    TimestampedPossessionState();

    /**
     * Updates the possession state
     *
     * @throws std::invalid_argument if update_time is older than timestamp()
     *
     * @param possessions The robots that have possession of the ball at update_time
     * @param update_time The timestamp to update possession state
     */
    void updateState(std::vector<RobotIdWithTeamSide> possessions, Timestamp update_time);

    /**
     * Returns the timestamp of when the possession state was last updated
     *
     * @return the timestamp
     */
    Timestamp timestamp() const;

    /**
     * Returns the robots that currently have possession of the ball. Usually, this will
     * be only one robot, but multiple robots could be fighting over the ball. If there
     * isn't enough data or if no robots have possession, then returns an empty vector
     *
     * @return The vector of RobotIdWithTeamSide with possession of the ball
     */
    const std::vector<RobotIdWithTeamSide> &getRobotsWithPossession() const;

    /**
     * Returns the Team Side that currently has exclusive possession of the ball
     *
     * @return TeamSide with possession or std::nullopt if it can't be determined
     */
    std::optional<TeamSide> getTeamWithPossession() const;

    /**
     * Returns the last time a robot had possession of the ball
     *
     * @param robot the robot to check
     *
     * @return last update time
     */
    std::optional<Timestamp> getLastPossessionTime(
        const RobotIdWithTeamSide &robot) const;

    /**
     * The equality operator for a TimestampedPossessionState. TimestampedPossessionState
     * are equal if the last possession maps and timestamps are equal
     *
     * @param other The TimestampedPossessionState to compare against for equality
     *
     * @return True if the other possession state is equal to this possession state, and
     * false otherwise
     */
    bool operator==(const TimestampedPossessionState &other) const;

    /**
     * The inequality operator for a TimestampedPossessionState.
     *
     * @param other The possession state to compare against for inequality
     *
     * @return True if the other possession state is not equal to this possession state,
     * and false otherwise
     */
    bool operator!=(const TimestampedPossessionState &other) const;

   private:
    std::map<RobotIdWithTeamSide, Timestamp> last_possession_map_;
    std::vector<RobotIdWithTeamSide> current_possessions_;
    Timestamp timestamp_;
};
