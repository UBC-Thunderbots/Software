#pragma once

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"

/**
 * Scores SupportTacticCandidates based on past performance and success of
 * each support tactic
 */
class SuccessScorer : public SupportTacticScorer
{
   public:
    explicit SuccessScorer() = default;

    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;
};
