#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

#include "software/util/generic_factory/generic_factory.h"

OffensivePlay::OffensivePlay(std::shared_ptr<Strategy> strategy,
                             std::unique_ptr<FeasibilityScorer> feasibility_scorer)
    : DynamicPlay(strategy, std::move(feasibility_scorer)),
      attacker_tactic_(std::make_shared<AttackerTactic>(strategy)),
      defense_play_(std::make_unique<DefensePlay>(strategy))
{
}

void OffensivePlay::updateTactics(const PlayUpdate& play_update)
{
    int num_defenders = std::min(play_update.num_tactics - 1, 2u);
    int num_supporters = play_update.num_tactics - num_defenders - 1;
    
    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, num_defenders,
        [&](PriorityTacticVector new_tactics) {
            for (auto& tactic_vec : new_tactics)
            {
                defense_tactics.insert(defense_tactics.end(), tactic_vec.begin(),
                                       tactic_vec.end());
            }
        },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));
    
    updateSupportTactics(num_supporters);

    play_update.set_tactics({{attacker_tactic_}, defense_tactics, support_tactics_});
}

static TGenericFactory<std::string, Play, OffensiveFriendlyThirdPlay,
                       std::shared_ptr<Strategy>>
    offensive_friendly_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveMiddleThirdPlay,
                       std::shared_ptr<Strategy>>
    offensive_middle_third_play_factory;

static TGenericFactory<std::string, Play, OffensiveEnemyThirdPlay,
                       std::shared_ptr<Strategy>>
    offensive_enemy_third_play_factory;
