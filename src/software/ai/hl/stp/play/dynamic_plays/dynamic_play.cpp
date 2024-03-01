#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"

DynamicPlay::DynamicPlay(bool requires_goalie,
                         std::shared_ptr<Strategy> strategy)
    : Play(requires_goalie, strategy),
      support_tactic_candidates_(allSupportTacticCandidates()),
      support_tactic_feasibility_scorer_(std::make_unique<FeasibilityScorer>()),
      support_tactic_duplication_scorer_(std::make_unique<DuplicationScorer>()),
      support_tactic_success_scorer_(std::make_unique<SuccessScorer>()),
      support_tactics_(),
      defense_play(std::make_unique<FSM<DefensePlayFSM>>(DefensePlayFSM(strategy->getAiConfig())))
{
}

void DynamicPlay::getNextTactics(TacticCoroutine::push_type &yield, const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
    while (true)
    {
        yield({{}});
    }
}

void DynamicPlay::updateTactics(const PlayUpdate &play_update)
{
    PriorityTacticVector tactics_to_return;

    PossessionStrategy possession_strategy = strategy->getPossessionStrategy(play_update.num_tactics);

    // Defense
    defense_play->updateControlParams(TbotsProto::MaxAllowedSpeedMode);
    defense_play->updateTactics(PlayUpdate(
                event.common.world_ptr, possession_strategy.num_defenders,
                [&tactics_to_return](PriorityTacticVector new_tactics)
                {
                    for (const auto& tactic_vector : new_tactics)
                    {
                        tactics_to_return.push_back(tactic_vector);
                    }
                },
                event.common.inter_play_communication,
                event.common.set_inter_play_communication_fun
    ));

    unsigned int num_support_tactics = possession_strategy.num_support;
    while (num_support_tactics > support_tactics_.size())
    {
        for (auto &candidate : support_tactic_candidates_)
        {
            candidate->clearScores();
            candidate->score(*support_tactic_feasibility_scorer_);
            candidate->score(*support_tactic_duplication_scorer_);
            candidate->score(*support_tactic_success_scorer_);
        }

        auto best_candidate = std::max_element(support_tactic_candidates_.begin(),
                                               support_tactic_candidates_.end());

        std::shared_ptr<Tactic> support_tactic = (*best_candidate)->createSupportTactic();
        support_tactics_.push_back(support_tactic);

        (*best_candidate)->updateScorer(*support_tactic_duplication_scorer_);
    }

    tactics_to_return.push_back({attacker_tactic_});
    tactics_to_return.push_back(support_tactics_);

    play_update.set_tactics(tactics_to_return);
}

std::vector<std::shared_ptr<SupportTacticCandidate>> allSupportTacticCandidates()
{
    return {std::make_shared<TypedSupportTacticCandidate<ReceiverTactic>>()};
}
