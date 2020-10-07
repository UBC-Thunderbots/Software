#include "software/world/world.h"

#include "boost/circular_buffer.hpp"
#include "software/parameter/dynamic_parameters.h"

World::World(const Field &field, const Ball &ball, const Team &friendly_team,
             const Team &enemy_team, unsigned int buffer_size)
    : field_(field),
      ball_(ball),
      friendly_team_(friendly_team),
      enemy_team_(enemy_team),
      current_game_state_(),
      // Store a small buffer of previous referee commands so we can filter out noise
      referee_command_history(REFEREE_COMMAND_BUFFER_SIZE),
      referee_stage_history(REFEREE_COMMAND_BUFFER_SIZE)
{
    // Grab the most recent timestamp from all of the members used to update the world
    last_update_timestamps.set_capacity(buffer_size);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateBallStateWithTimestamp(const TimestampedBallState &new_ball_state)
{
    ball_.updateState(new_ball_state);
    updateTimestamp(getMostRecentTimestampFromMembers());
    current_game_state_.updateBall(ball_);
}

void World::updateFriendlyTeamState(const Team &new_friendly_team_data)
{
    friendly_team_.updateState(new_friendly_team_data);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateEnemyTeamState(const Team &new_enemy_team_data)
{
    enemy_team_.updateState(new_enemy_team_data);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateTimestamp(Timestamp time_stamp)
{
    // Check if the timestamp buffer is empty
    if (last_update_timestamps.empty())
    {
        last_update_timestamps.push_front(time_stamp);
    }
    // Check that the new timestamp is not older than the most recent timestamp
    else if (time_stamp < getMostRecentTimestamp())
    {
        throw std::invalid_argument(
            "Error: Attempt tp update World state with old Timestamp");
    }
    else
    {
        last_update_timestamps.push_front(time_stamp);
    }
}

const Field &World::field() const
{
    return field_;
}

const Ball &World::ball() const
{
    return ball_;
}

const Team &World::friendlyTeam() const
{
    return friendly_team_;
}

const Team &World::enemyTeam() const
{
    return enemy_team_;
}

void World::updateRefereeCommand(const RefereeCommand &command)
{
    referee_command_history.push_back(command);
    // Take the consensus of the previous referee messages
    if (!referee_command_history.empty() &&
        std::all_of(
            referee_command_history.begin(), referee_command_history.end(),
            [&](auto gamestate) { return gamestate == referee_command_history.front(); }))
    {
        current_game_state_.updateRefereeCommand(command);
    }
}

void World::updateRefereeCommand(const RefereeCommand &command,
                                 Point ball_placement_point)
{
    updateRefereeCommand(command);
    current_game_state_.setBallPlacementPoint(ball_placement_point);
}

void World::updateRefereeStage(const RefereeStage &stage)
{
    referee_stage_history.push_back(stage);
    // Take the consensus of the previous referee messages
    if (!referee_stage_history.empty() &&
        std::all_of(referee_stage_history.begin(), referee_stage_history.end(),
                    [&](auto stage) { return stage == referee_stage_history.front(); }))
    {
        current_referee_stage_ = stage;
    }
}

Timestamp World::getMostRecentTimestampFromMembers()
{
    // Intit to 0.0. This way we will always get a larger or equal timestamp from one of
    // the members
    Timestamp most_recent_timestamp = Timestamp::fromSeconds(0.0);

    // Add all member timestamps to a list
    std::initializer_list<Timestamp> member_timestamps = {
        friendly_team_.getMostRecentTimestamp(), enemy_team_.getMostRecentTimestamp(),
        ball_.getPreviousStates().front().timestamp()};
    // Return the max

    return std::max(member_timestamps);
}

const Timestamp World::getMostRecentTimestamp() const
{
    return last_update_timestamps.front();
}

boost::circular_buffer<Timestamp> World::getTimestampHistory()
{
    return last_update_timestamps;
}

const GameState &World::gameState() const
{
    return current_game_state_;
}

bool World::operator==(const World &other) const
{
    return this->field() == other.field() && this->ball() == other.ball() &&
           this->friendlyTeam() == other.friendlyTeam() &&
           this->enemyTeam() == other.enemyTeam() &&
           this->gameState() == other.gameState();
}

bool World::operator!=(const World &other) const
{
    return !(*this == other);
}


void World::updateGameStateBall(const Ball &ball)
{
    current_game_state_.updateBall(ball);
}

void World::updateGameState(const GameState &game_state)
{
    current_game_state_ = game_state;
}

const RefereeStage &World::getRefereeStage() const
{
    return current_referee_stage_;
}
