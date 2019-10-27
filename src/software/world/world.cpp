#include "software/world/world.h"

#include "boost/circular_buffer.hpp"
#include "software/util/parameter/dynamic_parameters.h"

World::World()
    : World(Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0)),
            Ball(Point(), Vector(), Timestamp::fromSeconds(0)),
            Team(Duration::fromMilliseconds(
                Util::DynamicParameters->RobotExpiryBufferMilliseconds()->value())),
            Team(Duration::fromMilliseconds(
                Util::DynamicParameters->RobotExpiryBufferMilliseconds()->value())))
{
    // Set the default Timestamp as this parameter is not caught when using the World
    // contructor
    this->last_update_timestamps.push_front(Timestamp::fromSeconds(0.0));
}

World::World(const Field &field, const Ball &ball, const Team &friendly_team,
             const Team &enemy_team, unsigned int buffer_size)
    : field_(field),
      ball_(ball),
      friendly_team_(friendly_team),
      enemy_team_(enemy_team),
      game_state_(),
      // Store a small buffer of previous refbox game states so we can filter out noise
      refbox_game_state_history(3)
{
    // Grab the most recent timestamp from all of the members used to update the world
    last_update_timestamps.set_capacity(buffer_size);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateFieldGeometry(const Field &new_field_data)
{
    field_.updateDimensions(new_field_data);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateBallState(const Ball &new_ball_data)
{
    ball_.updateState(new_ball_data);
    updateTimestamp(getMostRecentTimestampFromMembers());
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

Field &World::mutableField()
{
    return field_;
}

const Ball &World::ball() const
{
    return ball_;
}

Ball &World::mutableBall()
{
    return ball_;
}

const Team &World::friendlyTeam() const
{
    return friendly_team_;
}

Team &World::mutableFriendlyTeam()
{
    return friendly_team_;
}

const Team &World::enemyTeam() const
{
    return enemy_team_;
}

Team &World::mutableEnemyTeam()
{
    return enemy_team_;
}

void World::updateRefboxGameState(const RefboxGameState &game_state)
{
    refbox_game_state_history.push_back(game_state);
    // Take the consensus of the previous refbox messages
    if (!refbox_game_state_history.empty() &&
        std::all_of(refbox_game_state_history.begin(), refbox_game_state_history.end(),
                    [&](auto gamestate) {
                        return gamestate == refbox_game_state_history.front();
                    }))
    {
        game_state_.updateRefboxGameState(game_state);
        game_state_.updateBall(ball_);
    }
    else
    {
        game_state_.updateRefboxGameState(game_state_.getRefboxGameState());
        game_state_.updateBall(ball_);
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
        ball_.getPreviousTimestamps().front(), field_.getMostRecentTimestamp()};
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
    return game_state_;
}

GameState &World::mutableGameState()
{
    return game_state_;
}
