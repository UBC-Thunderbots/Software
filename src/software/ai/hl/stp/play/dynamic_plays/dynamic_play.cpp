#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"

#include "software/util/generic_factory/generic_factory.h"

DynamicPlay::DynamicPlay(std::shared_ptr<Strategy> strategy)
    : Play(true, strategy),
      support_tactics_(),
      support_tactic_candidates_(allSupportTacticCandidates()),
      support_tactic_duplication_scorer_(std::make_unique<DuplicationScorer>()),
      support_tactic_success_scorer_(std::make_unique<SuccessScorer>())
{
}

void DynamicPlay::getNextTactics(TacticCoroutine::push_type &yield,
                                 const WorldPtr &world_ptr)
{
    // This function doesn't get called, it should be removed once coroutines
    // are phased out
    while (true)
    {
        yield({{}});
    }
}

void DynamicPlay::terminate(const WorldPtr &world_ptr)
{
    support_tactic_success_scorer_->evaluate(0);
    support_tactic_success_scorer_->reset();
    support_tactic_duplication_scorer_->reset();
    support_tactics_.clear();
}

void DynamicPlay::updateSupportTactics(unsigned int num_supporters)
{
    if (support_tactics_.size() > num_supporters)
    {
        support_tactics_.resize(num_supporters);
    }

    while (support_tactics_.size() < num_supporters)
    {
        for (auto &candidate : support_tactic_candidates_)
        {
            candidate->clearScores();
            candidate->score(*support_tactic_duplication_scorer_);
            candidate->score(*support_tactic_success_scorer_);
        }

        auto best_candidate = *std::max_element(support_tactic_candidates_.begin(),
                                                support_tactic_candidates_.end());

        std::shared_ptr<SupportTactic> support_tactic =
            best_candidate->createSupportTactic(strategy);
        support_tactics_.push_back(support_tactic);

        best_candidate->updateScorer(*support_tactic_duplication_scorer_);
        best_candidate->updateScorer(*support_tactic_success_scorer_);
    }
}

std::vector<std::shared_ptr<SupportTacticCandidate>> allSupportTacticCandidates()
{
    return {std::make_shared<TypedSupportTacticCandidate<ReceiverTactic>>()};
}
