#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"

DynamicPlay::DynamicPlay(TbotsProto::AiConfig ai_config, bool requires_goalie, std::shared_ptr<Strategy> strategy)
    : Play(ai_config, requires_goalie, strategy),
      support_tactic_candidates_(allSupportTacticCandidates()),
      support_tactic_feasibility_scorer_(std::make_unique<FeasibilityScorer>()),
      support_tactic_duplication_scorer_(std::make_unique<DuplicationScorer>()),
      support_tactic_success_scorer_(std::make_unique<SuccessScorer>()),
      support_tactics_()
{
}

void DynamicPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
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
    unsigned int num_support_tactics = play_update.num_tactics - 1;
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

    play_update.set_tactics({{attacker_tactic_}, support_tactics_});
}

std::vector<std::shared_ptr<SupportTacticCandidate>> allSupportTacticCandidates()
{
    return {std::make_shared<TypedSupportTacticCandidate<ReceiverTactic>>()};
}
