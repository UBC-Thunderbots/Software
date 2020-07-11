#include "software/world/timestamped_possession_state.h"


TimestampedPossessionState::TimestampedPossessionState()
    : timestamp_(Timestamp::fromSeconds(0.0))
{
}

void TimestampedPossessionState::updatePossessionState(
    const std::set<RobotIdWithTeamSide> &possessions, Timestamp update_time)
{
    timestamp_ = update_time;
    for (const auto &poss : possessions)
    {
        last_possession_map_[poss] = update_time;
    }
}

Timestamp TimestampedPossessionState::timestamp() const
{
    return timestamp_;
}

std::set<RobotIdWithTeamSide> TimestampedPossessionState::getRobotsWithCurrentPossession()
    const
{
    return current_possessions_;
}

std::optional<TeamSide> TimestampedPossessionState::getTeamWithMostRecentPossession()
    const
{
    auto possession_robots = getRobotsWithCurrentPossession();
    if (possession_robots.size() == 1)
    {
        return possession_robots.begin()->team_side;
    }
    else
    {
        return std::nullopt;
    }
}

bool TimestampedPossessionState::operator==(const TimestampedPossessionState &other) const
{
    return this->last_possession_map_ == other.last_possession_map_ &&
           this->current_possessions_ == other.current_possessions_ &&
           this->timestamp_ == other.timestamp_;
}

bool TimestampedPossessionState::operator!=(const TimestampedPossessionState &other) const
{
    return !(*this == other);
}
