#include "software/ai/hl/stp/play/offense/offense_play.h"

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/logger/logger.h"
#include "software/util/generic_factory/generic_factory.h"

OffensePlay::OffensePlay(TbotsProto::AiConfig config)
    : Play(config, true),
      shoot_or_pass_play(std::make_shared<ShootOrPassPlay>(ai_config)),
      defense_play(std::make_shared<DefensePlay>(ai_config)),
      possession_tracker(std::make_unique<PossessionTracker>())
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
    unsigned int num_defenders;

    TeamPossession possession = possession_tracker->getTeamWithPossession(
        play_update.world.friendlyTeam(), play_update.world.enemyTeam(),
        play_update.world.ball(), play_update.world.field());

    if (possession == TeamPossession::ENEMY_TEAM)
    {
        num_defenders = play_update.num_tactics;
    }
    else if (possession == TeamPossession::FRIENDLY_TEAM)
    {
        auto num_enemy_robots = play_update.world.enemyTeam().numRobots();

        if (play_update.num_tactics > num_enemy_robots)
        {
            // Always reduce number of defenders if we have more bots on the field than
            // the enemy team, so that we take advantage of the scoring opportunity
            // while the enemy team is down a robot
            num_defenders = 1;
        }
        else if (play_update.num_tactics <= 2)
        {
            // Set number of defenders to 0 so we don't get stuck defending and never 
            // attempt to score
            num_defenders = 0;
        }
        else 
        {
            num_defenders = 2;
        }
    }

    unsigned int num_shoot_or_pass = play_update.num_tactics - num_defenders;

    if (num_shoot_or_pass > 0)
    {
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
    }

    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    if (num_defenders > 0)
    {
        defense_play->updateTactics(PlayUpdate(
            play_update.world, num_defenders,
            [&tactics_to_return](PriorityTacticVector new_tactics) {
                for (const auto &tactic_vector : new_tactics)
                {
                    tactics_to_return.push_back(tactic_vector);
                }
            },
            play_update.inter_play_communication,
            play_update.set_inter_play_communication_fun));
    }

    play_update.set_tactics(tactics_to_return);
}

std::vector<std::string> OffensePlay::getState()
{
    std::vector<std::string> state = {objectTypeName(*this)};
    auto spp_state                 = shoot_or_pass_play->getState();
    auto d_state                   = defense_play->getState();
    state.insert(state.end(), spp_state.begin(), spp_state.end());
    state.insert(state.end(), d_state.begin(), d_state.end());
    return state;
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, OffensePlay, TbotsProto::AiConfig> factory;
