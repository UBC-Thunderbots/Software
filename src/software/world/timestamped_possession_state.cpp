#include "software/world/timestamped_possession_state.h"


TimestampedPossessionState::TimestampedPossessionState()
    : timestamp_(Timestamp::fromSeconds(0.0))
{
}

void TimestampedPossessionState::updateState(std::vector<RobotIdWithTeamSide> possessions,
                                             Timestamp update_time)
{
    if (update_time < timestamp_)
    {
        throw std::invalid_argument("Update time is older than the last update time");
    }
    timestamp_           = update_time;
    current_possessions_ = possessions;
    for (const auto &poss : possessions)
    {
        last_possession_map_[poss] = update_time;
    }
}

Timestamp TimestampedPossessionState::timestamp() const
{
    return timestamp_;
}

const std::vector<RobotIdWithTeamSide>
    &TimestampedPossessionState::getRobotsWithPossession() const
{
    return current_possessions_;
}

std::optional<TeamSide> TimestampedPossessionState::getTeamWithPossession() const
{
    if (current_possessions_.size() == 1)
    {
        return current_possessions_.begin()->team_side;
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
