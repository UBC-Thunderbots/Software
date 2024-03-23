#pragma once

#include "software/ai/evaluation/scoring/support_tactics/duplication_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/feasibility_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/success_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/hl/stp/play/play.h"

/**
 * A DynamicPlay is a Play that assigns tactics based on scoring functions
 * whose parameters and outputs are incrementally adjusted over time based
 * on the Play's success. This enables the Play to "learn" which tactics are
 * most effective for a given gameplay scenario.
 * 
 * Every time we select and run the DynamicPlay, we call it an iteration of the play.
 * At the start of each iteration, the DynamicPlay chooses which support tactics
 * to run and commits to those tactics for the length of the iteration. 
 * When the iteration ends, the DynamicPlay's performance over the iteration
 * is evaluated, and the assessment is used to either encourage or discourage
 * selection of the iteration's chosen support tactics in future iterations. 
 */
class DynamicPlay : public Play
{
   public:
    /**
     * Evaluate the DynamicPlay and start a new iteration
     * 
     * @param score score between [-1, 1] rating the success of the DynamicPlay
     * for the current iteration
     */
    virtual void evaluate(double score);

   protected:
    /**
     * Base constructor for DynamicPlay
     *
     * @param strategy the Strategy
     * @param feasibility_scorer the feasibility scorer for the play
     */
    explicit DynamicPlay(std::shared_ptr<Strategy> strategy,
                         std::unique_ptr<FeasibilityScorer> feasibility_scorer);

    DynamicPlay() = delete;

    // TODO (#2359): delete once all plays are not coroutines
    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;

    /**
     * Selects and adds support tactics to the `support_tactics_` vector 
     * until the vector has at least the specified number of support tactics 
     *
     * @param num_supporters the number of support tactics
     */
    void updateSupportTactics(unsigned int num_supporters);

    std::vector<std::shared_ptr<Tactic>> support_tactics_;
    std::vector<std::shared_ptr<SupportTacticCandidate>> support_tactic_candidates_;

    std::unique_ptr<FeasibilityScorer> support_tactic_feasibility_scorer_;
    std::unique_ptr<DuplicationScorer> support_tactic_duplication_scorer_;
    std::unique_ptr<SuccessScorer> support_tactic_success_scorer_;
};

/**
 * Returns all SupportTacticCandidates eligible for scoring and assignment
 * in a DynamicPlay
 *
 * @return a vector of shared pointers to all SupportTacticCandidates
 */
std::vector<std::shared_ptr<SupportTacticCandidate>> allSupportTacticCandidates();
