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

void World::updateFieldGeometry(const Field &new_field_data)
{
    field_.updateDimensions(new_field_data);
}

void World::updateBallState(const thunderbots_msgs::Ball &new_ball_msg)
{
    ball_.update(new_ball_msg);
}

void World::updateBallState(const Ball &new_ball_data)
{
    ball_.update(new_ball_data);
}

void World::updateFriendlyTeam(const thunderbots_msgs::Team &new_friendly_team_msg)
{
    friendly_team_.update(new_friendly_team_msg);
}

void World::updateFriendlyTeam(const Team &new_friendly_team_data)
{
    friendly_team_.update(new_friendly_team_data);
}

void World::updateEnemyTeam(const thunderbots_msgs::Team &new_enemy_team_msg)
{
    enemy_team_.update(new_enemy_team_msg);
}

void World::updateEnemyTeam(const Team &new_enemy_team_data)
{
    enemy_team_.update(new_enemy_team_data);
}

void World::clearFriendlyTeamRobots()
{
    friendly_team_.clearAllRobots();
}

void World::clearEnemyTeamRobots()
{
    enemy_team_.clearAllRobots();
}

const Field &World::field() const
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
