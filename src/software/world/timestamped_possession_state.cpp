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
    timestamp_ = update_time;
    current_friendly_possessions_.clear();
    current_enemy_possessions_.clear();
    for (const auto &poss : possessions)
    {
        last_possession_map_[poss] = update_time;
        if (poss.team_side == TeamSide::FRIENDLY)
        {
            current_friendly_possessions_.push_back(poss.id);
        }
        if (poss.team_side == TeamSide::ENEMY)
        {
            current_enemy_possessions_.push_back(poss.id);
        }
    }
}

Timestamp TimestampedPossessionState::lastUpdateTimestamp() const
{
    return timestamp_;
}

const std::vector<RobotId> &TimestampedPossessionState::getFriendlyRobotsWithPossession()
    const
{
    return current_friendly_possessions_;
}

const std::vector<RobotId> &TimestampedPossessionState::getEnemyRobotsWithPossession()
    const
{
    return current_enemy_possessions_;
}

std::optional<TeamSide> TimestampedPossessionState::getTeamWithExclusivePossession() const
{
    if (current_friendly_possessions_.size() == 1 &&
        current_enemy_possessions_.size() == 0)
    {
        return TeamSide::FRIENDLY;
    }

    if (current_enemy_possessions_.size() == 1 &&
        current_friendly_possessions_.size() == 0)
    {
        return TeamSide::ENEMY;
    }
    return std::nullopt;
}

std::optional<Timestamp> TimestampedPossessionState::getLastPossessionTime(
    const RobotIdWithTeamSide &robot) const
{
    auto last_possession_it = last_possession_map_.find(robot);
    if (last_possession_it == last_possession_map_.end())
    {
        return std::nullopt;
    }
    else
    {
        return last_possession_it->second;
    }
}

bool TimestampedPossessionState::operator==(const TimestampedPossessionState &other) const
{
    return this->last_possession_map_ == other.last_possession_map_ &&
           this->current_friendly_possessions_ == other.current_friendly_possessions_ &&
           this->current_enemy_possessions_ == other.current_enemy_possessions_;
}

bool TimestampedPossessionState::operator!=(const TimestampedPossessionState &other) const
{
    return !(*this == other);
}
