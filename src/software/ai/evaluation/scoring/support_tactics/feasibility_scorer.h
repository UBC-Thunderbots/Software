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

    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;
};
