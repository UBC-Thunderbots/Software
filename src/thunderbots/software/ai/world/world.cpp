#include "world.h"

World::World()
    : field_(Field()), ball_(Ball()), friendly_team_(Team()), enemy_team_(Team())
{
}

World::World(
    const Field &field, const Ball &ball, const Team &friendly_team,
    const Team &enemy_team)
    : field_(field), ball_(ball), friendly_team_(friendly_team), enemy_team_(enemy_team)
{
}

// TODO: Should we be able to update the field and other objects internally?
void World::updateField(const Field &new_field)
{
    field_ = new_field;
}

void World::updateBall(const Ball &new_ball)
{
    ball_ = new_ball;
}

void World::updateFriendlyTeam(const Team &new_friendly_team)
{
    friendly_team_ = new_friendly_team;
}

void World::updateEnemyTeam(const Team &new_enemy_team)
{
    enemy_team_ = new_enemy_team;
}

Ball World::ball() const
{
    return ball_;
}

Field World::field() const
{
    return field_;
}

Team World::friendlyTeam() const
{
    return friendly_team_;
}

Team World::enemyTeam() const
{
    return enemy_team_;
}
