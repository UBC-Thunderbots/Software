#include "software/ai/hl/stp/play/dynamic_plays/offensive_plays.h"

#include "software/util/generic_factory/generic_factory.h"

void OffensivePlay::evaluate()
{
    DynamicPlay::evaluate();
    attacker_tactic_->evaluate();
}

void OffensivePlay::updateTactics(const PlayUpdate &play_update)
{
    DynamicPlay::updateTactics();

    PossessionStrategy possession_strategy =
        (*strategy)->getPossessionStrategy(play_update.num_tactics);

    std::vector<std::shared_ptr<Tactic>> defense_tactics;
    defense_play_->updateControlParams(TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);
    defense_play_->updateTactics(PlayUpdate(
        play_update.world_ptr, possession_strategy.num_defenders,
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
