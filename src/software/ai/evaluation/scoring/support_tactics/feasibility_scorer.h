#pragma once

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"

/**
 * Scores SupportTacticCandidates based on their feasibility for a specific play
 * and gameplay scenario
 */
class FeasibilityScorer : public SupportTacticScorer
{
   public:
    explicit FeasibilityScorer() = default;

    /**
     * The javadoc comment for all `score` methods below can be read as:
     * Visits the given SupportTacticCandidate to score it
     *
     * @param candidate the SupportTacticCandidate to score
     *
     * @return the score for the SupportTacticCandidate, in the range [-1.0, 1.0]
     */
    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;
};
