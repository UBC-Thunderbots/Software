#include "ai.h"

AI::AI() : navigator(std::make_unique<RRTNav>()), high_level(std::make_unique<STP_HL>())
{
}

std::vector<std::unique_ptr<Primitive>> AI::getPrimitives(
    const AITimestamp &timestamp) const
{
    // NOTE: The only thing the AI really needs an updated timestamp for (updated relative
    // to the timestamp when the state/world was last updated) is to update the
    // "zero-time"
    // for any predictors (ie. modules that predict Robot, Ball position etc.). When those
    // are implemented we can update them in some way from here, if necessary.

    std::vector<std::unique_ptr<Intent>> assignedIntents =
        high_level->getIntentAssignment(world);

    std::vector<std::unique_ptr<Primitive>> assignedPrimitives =
        navigator->getAssignedPrimitives(world, assignedIntents);

    return assignedPrimitives;
}

void AI::updateWorldBallState(const thunderbots_msgs::Ball &new_ball_msg)
{
    world.ball.update(new_ball_msg);
}

void AI::updateWorldFieldState(const thunderbots_msgs::Field &new_field_msg)
{
    world.field.updateDimensions(new_field_msg);
}

void AI::updateWorldFriendlyTeamState(const thunderbots_msgs::Team &new_friendly_team_msg)
{
    world.friendly_team.update(new_friendly_team_msg);
}

void AI::updateWorldEnemyTeamState(const thunderbots_msgs::Team &new_enemy_team_msg)
{
    world.enemy_team.update(new_enemy_team_msg);
}
