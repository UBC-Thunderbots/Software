#pragma once

#include "software/ai/hl/stp/play/play.h"

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.hpp"

class DynamicPlay : public Play {
   public:
    /**
     * Creates a new DynamicPlay
     *
     * @param ai_config The AI configuration
     * @param requires_goalie Whether this play requires a goalie
     */
    explicit DynamicPlay(TbotsProto::AiConfig ai_config, bool requires_goalie);

   protected:
    // TODO (#2359): delete once all plays are not coroutines
    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    void updateTactics(const PlayUpdate &play_update) override;

    SupportTacticCandidateVector support_tactic_candidates_;

    std::unique_ptr<FeasibilityScorer> support_tactic_feasibility_scorer_;
    std::unique_ptr<DuplicationScorer> support_tactic_duplication_scorer_;
    std::unique_ptr<SuccessScorer> support_tactic_success_scorer_;

    std::unique_ptr<AttackerTactic> attacker_tactic_;
    std::vector<std::unique_ptr<Tactic>> support_tactics_;
}