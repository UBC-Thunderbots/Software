#pragma once

#include "software/ai/hl/stp/play/play.h"

#include "software/ai/evaluation/scoring/support_tactics/duplication_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/success_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"

/**
 * A DynamicPlay is a Play that assigns tactics based on scoring functions
 * whose parameters and outputs are incrementally adjusted over time based 
 * on the Play's success. This enables the Play to "learn" which tactics are 
 * most effective for a given gameplay scenario.
 */
class DynamicPlay : public Play 
{
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

    std::shared_ptr<Tactic> attacker_tactic_;
    std::vector<std::shared_ptr<Tactic>> support_tactics_;
};
