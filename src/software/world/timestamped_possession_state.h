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
};

class TimestampedPossessionState
{
   public:
    /**
     * Creates a new possession state that tracks no robots and has timestamp of 0
     */
    TimestampedPossessionState();

    /**
     * Update the possession state
     *
     * @param possessions The robots that have possession of the ball at update_time
     * @param update_time The timestamp to update possession state
     */
    void updatePossessionState(const std::vector<RobotIdWithTeamSide> &possessions,
                               Timestamp update_time);

    /**
     * Returns the timestamp of when the possession state was last updated
     *
     * @return the timestamp
     */
    Timestamp timestamp() const;

    /**
     * Returns the Robot that most recently or currently has possession of the ball
     *
     * @return RobotIdWithTeamSide with possession or std::nullopt if it can't be
     * determined
     */
    std::optional<RobotIdWithTeamSide> getRobotWithMostRecentPossession() const;

    /**
     * Returns the Team Side that most recently or currently has possession of the ball
     *
     * @return TeamSide with possession or std::nullopt if it can't be determined
     */
    std::optional<TeamSide> getTeamWithMostRecentPossession() const;

    /**
     * The equality operator for a TimestampedPossessionState.
     * TimestampedPossessionState are equal if the last possession maps and timestamps are
     * equal
     *
     * @param other The TimestampedPossessionState to compare against for equality
     * @return True if the other possession state is equal to this possession state, and
     * false otherwise
     */
    bool operator==(const TimestampedPossessionState &other) const;

    /**
     * The inequality operator for a TimestampedPossessionState.
     *
     * @param other The possession state to compare against for inequality
     * @return True if the other possession state is not equal to this possession state,
     * and false otherwise
     */
    bool operator!=(const TimestampedPossessionState &other) const;

   private:
    std::map<RobotIdWithTeamSide, Timestamp> last_possession_map_;
    Timestamp timestamp_;
};
