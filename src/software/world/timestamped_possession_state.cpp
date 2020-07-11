#include "software/world/timestamped_possession_state.h"


TimestampedPossessionState::TimestampedPossessionState()
    : last_possession_map_(std::map<RobotIdWithTeamSide, Timestamp>()),
      timestamp_(Timestamp::fromSeconds(0.0))
{
}

void TimestampedPossessionState::updatePossessionState(
    const std::vector<RobotIdWithTeamSide> &possessions, Timestamp update_time)
{
    // TODO: update this properly
}

Timestamp TimestampedPossessionState::timestamp() const
{
    return timestamp_;
}

std::optional<RobotIdWithTeamSide>
TimestampedPossessionState::getRobotWithMostRecentPossession() const
{
    // TODO: calculate this properly
    return std::nullopt;
}

std::optional<TeamSide> TimestampedPossessionState::getTeamWithMostRecentPossession()
    const
{
    auto possession_robot = getRobotWithMostRecentPossession();
    if (possession_robot)
    {
        return possession_robot->team_side;
    }
    return std::nullopt;
}

bool TimestampedPossessionState::operator==(const TimestampedPossessionState &other) const
{
    return this->last_possession_map_ == other.last_possession_map_ &&
           this->timestamp_ == other.timestamp_;
}

bool TimestampedPossessionState::operator!=(const TimestampedPossessionState &other) const
{
    return !(*this == other);
}
