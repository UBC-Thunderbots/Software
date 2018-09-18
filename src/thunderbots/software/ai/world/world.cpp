#include "world.h"

World::World()
    : field_(),
      ball_(),
      friendly_team_(std::chrono::milliseconds(0)),
      enemy_team_(std::chrono::milliseconds(0))
{
}

World::World(const Field &field, const Ball &ball, const Team &friendly_team,
             const Team &enemy_team)
    : field_(field), ball_(ball), friendly_team_(friendly_team), enemy_team_(enemy_team)
{
}

void World::updateFieldGeometry(const Field &new_field_data)
{
    field_.updateDimensions(new_field_data);
}

void World::updateBallState(const Ball &new_ball_data)
{
    ball_.updateState(new_ball_data);
}

void World::updateFriendlyTeam(const Team &new_friendly_team_data,
                               const std::chrono::steady_clock::time_point timestamp)
{
    friendly_team_.updateState(new_friendly_team_data, timestamp);
}

void World::updateEnemyTeam(const Team &new_enemy_team_data,
                            const std::chrono::steady_clock::time_point timestamp)
{
    enemy_team_.updateState(new_enemy_team_data, timestamp);
}

const Field &World::field()
{
    return field_;
}

const Ball &World::ball() const
{
    return ball_;
}

const Team &World::friendly_team() const
{
    return friendly_team_;
}

const Team &World::enemy_team() const
{
    return enemy_team_;
}
