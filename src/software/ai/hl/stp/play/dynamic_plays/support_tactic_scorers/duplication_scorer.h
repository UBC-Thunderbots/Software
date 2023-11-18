#pragma once

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_candidate.hpp"

/**
 * Scores SupportTacticCandidates based on how many times they have already been selected
 */
class DuplicationScorer : public SupportTacticScorer
{
   public:
    explicit DuplicationScorer();

    /**
     * Records a usage of the given support tactic candidate
     *
     * @param candidate the support tactic candidate that was selected
     */
    void recordCandidateSelection(const SupportTacticCandidate<Tactic> &candidate);

    double score(const SupportTacticCandidate<PassReceiverTactic> &candidate) override;
    double score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate) override;
    double score(const SupportTacticCandidate<CherryPickerTactic> &candidate) override;
    double score(const SupportTacticCandidate<DisrupterTactic> &candidate) override;

   private:
    std::map<SupportTacticCandidate<Tactic>, int> selection_counter_;
}