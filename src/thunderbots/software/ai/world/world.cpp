#include "world.h"

#include <util/parameter/dynamic_parameters.h>

#include "boost/circular_buffer.hpp"

World::World()
    : World(Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0)),
            Ball(Point(), Vector(), Timestamp::fromSeconds(0)),
            Team(Duration::fromMilliseconds(
                Util::DynamicParameters::robot_expiry_buffer_milliseconds.value())),
            Team(Duration::fromMilliseconds(
                Util::DynamicParameters::robot_expiry_buffer_milliseconds.value())))
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
      game_state_()
{
    // Grab the most recent timestamp from all of the members used to update the world
    last_update_timestamps.set_capacity(buffer_size);
    updateTimestamp(getMostRecentTimestampFromMembers());
}

void World::updateFieldGeometry(const Field &new_field_data)
{
    updateTimestamp(new_field_data.getMostRecentTimestamp());
    field_.updateDimensions(new_field_data);
}

void World::updateBallState(const Ball &new_ball_data)
{
    updateTimestamp(new_ball_data.getPreviousTimestamps().front());
    ball_.updateState(new_ball_data);
}

void World::updateFriendlyTeamState(const Team &new_friendly_team_data)
{
    updateTimestamp(new_friendly_team_data.getMostRecentTimestamp());
    friendly_team_.updateState(new_friendly_team_data);
}

void World::updateEnemyTeamState(const Team &new_enemy_team_data)
{
    updateTimestamp(enemy_team_.getMostRecentTimestamp());
    enemy_team_.updateState(new_enemy_team_data);
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
    game_state_.updateRefboxGameState(game_state);
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
