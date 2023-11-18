#pragma once

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.hpp"

/**
 * Scores SupportTacticCandidates based on their feasibility for a specific play
 * and gameplay scenario
 */
class FeasibilityScorer : public SupportTacticScorer
{
   public:
    explicit FeasibilityScorer() = default;

    virtual double score(const SupportTacticCandidate<PassReceiverTactic> &candidate) override;
    virtual double score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate) override;
    virtual double score(const SupportTacticCandidate<CherryPickerTactic> &candidate) override;
    virtual double score(const SupportTacticCandidate<DisrupterTactic> &candidate) override;
}