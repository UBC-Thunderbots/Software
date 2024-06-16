#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

#include "software/util/generic_factory/generic_factory.h"

OffensivePlay::OffensivePlay(std::shared_ptr<Strategy> strategy,
                             std::unique_ptr<FeasibilityScorer> feasibility_scorer,
                             std::shared_ptr<AttackerTactic> attacker_tactic)
    : DynamicPlay(strategy, std::move(feasibility_scorer)),
      attacker_tactic_(attacker_tactic),
      defense_play_(std::make_unique<DefensePlay>(strategy))
{
}

void OffensivePlay::terminate(const WorldPtr& world_ptr)
{
    DynamicPlay::terminate(world_ptr);
    attacker_tactic_->terminate(world_ptr);
}

void OffensivePlay::updateTactics(const PlayUpdate& play_update)
{
    PriorityTacticVector tactics;
    tactics.reserve(play_update.num_tactics);

    const bool is_attacker_suspended =
        attacker_tactic_->tryResumingIfSuspended(play_update.world_ptr);

    // The AttackerTactic should only be returned if it has not suspended
    // execution of its current skill (it will not yield primitives if suspended)
    if (!is_attacker_suspended)
    {
        tactics.push_back({attacker_tactic_});
    }

    int num_attackers  = static_cast<unsigned int>(tactics.size());
    int num_defenders  = std::min(play_update.num_tactics - num_attackers, 2u);
    int num_supporters = play_update.num_tactics - num_attackers - num_defenders;

    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, num_defenders,
        [&](PriorityTacticVector new_tactics)
        {
            for (auto& tactic_vec : new_tactics)
            {
                defense_tactics.insert(defense_tactics.end(), tactic_vec.begin(),
                                       tactic_vec.end());
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

    updateSupportTactics(num_supporters);

    tactics.push_back(defense_tactics);
    tactics.push_back(support_tactics_);

    play_update.set_tactics(tactics);
}

static TGenericFactory<std::string, Play, OffensiveFriendlyThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_friendly_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveMiddleThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_middle_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveEnemyThirdPlay,
                       std::shared_ptr<Strategy>, std::shared_ptr<AttackerTactic>>
    offensive_enemy_third_play_factory;
