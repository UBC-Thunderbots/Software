#pragma once

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.hpp"

class SuccessScorer : public SupportTacticScorer
{
   public:
    explicit SuccessScorer() = default;

    double score(const SupportTacticCandidate<PassReceiverTactic> &candidate) override;
    double score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate) override;
    double score(const SupportTacticCandidate<CherryPickerTactic> &candidate) override;
    double score(const SupportTacticCandidate<DisrupterTactic> &candidate) override;
}