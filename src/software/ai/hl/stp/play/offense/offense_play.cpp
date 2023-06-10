#include "software/ai/hl/stp/play/offense/offense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/possession.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

OffensePlay::OffensePlay(TbotsProto::AiConfig config)
    : Play(config, true),
      shoot_or_pass_play(std::make_shared<ShootOrPassPlay>(ai_config)),
      crease_defense_play(std::make_shared<CreaseDefensePlay>(ai_config))
{
}

void OffensePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // This function doesn't get called so it does nothing
    while (true)
    {
        yield({{}});
    }
}

void OffensePlay::updateTactics(const PlayUpdate &play_update)
{
    auto num_friendly_robots = play_update.num_tactics;
    auto num_enemy_robots    = play_update.world.enemyTeam().numRobots();

    if (num_friendly_robots == 0)
    {
        return;
    }

    PriorityTacticVector tactics_to_return;
    unsigned int num_defenders, num_attackers;

    // Set single touch modes for free kick and kickoff
    if ((play_update.world.gameState().isSetupState() &&
         (play_update.world.gameState().isOurDirectFree() ||
          play_update.world.gameState().isOurIndirectFree())) ||
        (play_update.world.gameState().isReadyState() &&
         play_update.world.gameState().isOurKickoff()))
    {
        shoot_or_pass_play->updateControlParams(true);
    }

    // Assign number of defenders
    if (num_friendly_robots > num_enemy_robots)
    {
        // If we have more robots than the enemy, then we can reduce the number of
        // defenders
        num_defenders = 1;
        if (num_enemy_robots < 2)
        {
            // If the enemy only has 1 robot, then we can dedicate all robots to offense
            num_defenders = 0;
        }
    }
    else if (num_friendly_robots <= 3)
    {
        // If we are short on robots and the enemy is not, then increase the number of
        // defenders
        num_defenders = num_friendly_robots - 1;
    }
    else if (play_update.world.ball().position().x() < 0)
    {
        // If we have more than 3 robots and the ball is on our side of the field, then
        // dedicate an additional robot to defense
        num_defenders = 3;
    }
    else
    {
        num_defenders = 2;
    }

    num_attackers = num_friendly_robots - num_defenders;

    shoot_or_pass_play->updateTactics(PlayUpdate(
        play_update.world, num_attackers,
        [&tactics_to_return](PriorityTacticVector new_tactics) {
            for (const auto &tactic_vector : new_tactics)
            {
                tactics_to_return.push_back(tactic_vector);
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

    crease_defense_play->updateControlParams(
        play_update.world.ball().position(),
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    crease_defense_play->updateTactics(PlayUpdate(
        play_update.world, num_defenders,
        [&tactics_to_return](PriorityTacticVector new_tactics) {
            for (const auto &tactic_vector : new_tactics)
            {
                tactics_to_return.push_back(tactic_vector);
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

    play_update.set_tactics(tactics_to_return);
}

std::vector<std::string> OffensePlay::getState()
{
    std::vector<std::string> state = {objectTypeName(*this)};
    auto spp_state                 = shoot_or_pass_play->getState();
    auto cd_state                  = crease_defense_play->getState();
    state.insert(state.end(), spp_state.begin(), spp_state.end());
    state.insert(state.end(), cd_state.begin(), cd_state.end());
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, OffensePlay, TbotsProto::AiConfig> factory;
