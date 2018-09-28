#include "world.h"

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

void World::updateFriendlyTeamState(const Team &new_friendly_team_data)
{
    friendly_team_.updateState(new_friendly_team_data);
}

void World::updateEnemyTeamState(const Team &new_enemy_team_data)
{
    enemy_team_.updateState(new_enemy_team_data);
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
