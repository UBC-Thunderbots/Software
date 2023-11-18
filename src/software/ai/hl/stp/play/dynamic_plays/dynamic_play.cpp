#include "software/ai/hl/stp/play/dynamic_plays/dynamic_play.h"

DynamicPlay::DynamicPlay(TbotsProto::AiConfig ai_config, bool requires_goalie) 
    : Play(ai_config, requires_goalie),
      support_tactic_candidates_(allSupportTacticCandidates()),  
      support_tactic_feasibility_scorer_(std::make_unique<FeasibilityScorer>()),
      support_tactic_duplication_scorer_(std::make_unique<DuplicationScorer>()),
      support_tactic_success_scorer_(std::make_unique<SuccessScorer>()),
      attacker_tactic_(std::make_unique<AttackerTactic>()),
      support_tactics_({})
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
        for (const SupportTacticCandidate<Tactic> &candidate : support_tactic_candidates_) 
        {
            candidate.resetTotalScore();
            candidate.score(support_tactic_feasibility_scorer_);
            candidate.score(support_tactic_duplication_scorer_);
            candidate.score(support_tactic_success_scorer_);
        }

        auto best_candidate = std::max_element(
            support_tactic_candidates_.begin(), support_tactic_candidates_.end(),
            [](const SupportTacticCandidate<Tactic> &a, SupportTacticCandidate<Tactic> &b) { 
                return a.getTotalScore() < b.getTotalScore(); 
            });

        std::shared_ptr<Tactic> support_tactic = best_candidate->createSupportTactic();
        support_tactics_.push_back(support_tactic);

        support_tactic_duplication_scorer_->recordTacticUsage(best_candidate);
    }

    play_update.set_tactics({{attacker_tactic}, support_tactics_}});
}