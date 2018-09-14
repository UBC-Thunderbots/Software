#include "world.h"

World::World() : field_(), ball_(), friendly_team_(), enemy_team_()
{
}

World::World(const Field &field, const Ball &ball, const Team &friendly_team,
             const Team &enemy_team)
    : field_(field), ball_(ball), friendly_team_(friendly_team), enemy_team_(enemy_team)
{
}

void World::updateFieldGeometry(const thunderbots_msgs::Field &new_field_msg)
{
    field_.updateDimensions(new_field_msg);
}

void World::updateBallState(const Ball &new_ball_data)
{
    ball_.update(new_ball_data);
}

void World::updateFriendlyTeam(const thunderbots_msgs::Team &new_friendly_team_msg)
{
    friendly_team_.update(new_friendly_team_msg);
}

void World::updateEnemyTeam(const thunderbots_msgs::Team &new_enemy_team_msg)
{
    enemy_team_.update(new_enemy_team_msg);
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
