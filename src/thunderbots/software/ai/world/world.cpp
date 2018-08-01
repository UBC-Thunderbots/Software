#include "world.h"

World::World()
    : field(Field()), ball(Ball()), friendly_team(Team()), enemy_team(Team())
{
}

World::World(
    const Field &field, const Ball &ball, const Team &friendly_team,
    const Team &enemy_team)
    : field(field), ball(ball), friendly_team(friendly_team), enemy_team(enemy_team)
{
}
