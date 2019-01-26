#include "ai.h"

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

AI::AI(const World &world)
    : world(world),
      navigator(std::make_unique<RRTNav>()),
      high_level(std::make_unique<STP_HL>())
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

void AI::updateWorldBallState(const Ball &new_ball_data)
{
    world.updateBallState(new_ball_data);
}

void AI::updateWorldFieldState(const Field &new_field_data)
{
    world.updateFieldGeometry(new_field_data);
}

void AI::updateWorldFriendlyTeamState(const Team &new_friendly_team_data)
{
    world.updateFriendlyTeamState(new_friendly_team_data);
}

void AI::updateWorldEnemyTeamState(const Team &new_enemy_team_data)
{
    world.updateEnemyTeamState(new_enemy_team_data);
}

void AI::updateWorldRefboxGameState(const RefboxGameState &game_state)
{
    // TODO: delete this logging message
    if (game_state != world.gameState().getRefboxGameState())
    {
        LOG(INFO) << "Game state changed to: " << game_state;
    }
    world.updateRefboxGameState(game_state);
}
