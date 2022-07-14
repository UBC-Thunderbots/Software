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
    if (play_update.num_tactics == 0)
    {
        return;
    }

    PriorityTacticVector tactics_to_return;
    unsigned int num_defenders = 2;
    unsigned int num_enemy_robots =
        static_cast<unsigned int>(play_update.world.enemyTeam().numRobots());

    if (play_update.world.gameState().isSetupState() &&
        (play_update.world.gameState().isOurDirectFree() ||
         play_update.world.gameState().isOurIndirectFree()))
    {
        // if we're in free kick on the enemy side, then we can commit more robots in
        // forward positions
        if (play_update.world.ball().position().x() > 0)
        {
            num_defenders = static_cast<unsigned int>(std::max(
                0, static_cast<int>(
                       2 - 2 * play_update.world.ball().position().x() /
                               (play_update.world.field().fieldLines().xMax() / 3.0))));
        }

        shoot_or_pass_play->updateControlParams(true);
    }
    else if (play_update.world.gameState().isReadyState() &&
             play_update.world.gameState().isOurKickoff())
    {
        shoot_or_pass_play->updateControlParams(true);
    }
    else if (num_enemy_robots <= 3)
    {
        // enemy team has at most half a full team, so we need at most half the number of
        // defenders
        num_defenders = 1;
        if (num_enemy_robots < 2)
        {
            num_defenders = 0;
        }
    }
    else if (play_update.num_tactics <= 3)
    {
        // play_update.num_tactics == 0 is handled above
        num_defenders = play_update.num_tactics - 1;
    }
    else if (play_update.world.ball().position().x() <
             play_update.world.field().fieldLines().xMin() / 4.0)
    {
        // if we have more than 3 robots and the ball is on our side of the field, then we
        // should dedicate an additional robot to defense
        num_defenders = 3;
    }

    unsigned int num_shoot_or_pass = play_update.num_tactics - num_defenders;

    shoot_or_pass_play->updateTactics(PlayUpdate(
        play_update.world, num_shoot_or_pass,
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
