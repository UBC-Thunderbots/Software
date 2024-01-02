#pragma once

//#include <map>

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"

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
    void recordCandidateSelection(const SupportTacticCandidate &candidate);

    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;

   private:
    // std::map<SupportTacticCandidate, int> selection_counter_;
};
