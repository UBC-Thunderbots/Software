#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

#include "software/util/generic_factory/generic_factory.h"

OffensivePlay::OffensivePlay(std::shared_ptr<Strategy> strategy,
                             std::unique_ptr<FeasibilityScorer> feasibility_scorer)
    : DynamicPlay(strategy, std::move(feasibility_scorer)),
      attacker_tactic_(std::make_shared<AttackerTactic>(strategy)),
      defense_play_(std::make_unique<DefensePlay>(strategy))
{
}

void OffensivePlay::evaluate(double score)
{
    DynamicPlay::evaluate(score);
    attacker_tactic_->evaluate(score);
}

void OffensivePlay::updateTactics(const PlayUpdate &play_update)
{
    PossessionStrategy possession_strategy =
        (*strategy)->getPossessionStrategy(play_update.num_tactics);

    updateSupportTactics(possession_strategy.supporters);

    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, possession_strategy.defenders,
        [&](PriorityTacticVector new_tactics) { defense_tactics = new_tactics },
        play_update.inter_play_communication,
        play_update.set_inter_play_communication_fun));

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
