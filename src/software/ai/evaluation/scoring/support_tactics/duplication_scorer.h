#pragma once

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/util/type_map/type_map.h"

/**
 * Scores SupportTacticCandidates based on how many times they have already been selected
 */
class DuplicationScorer : public SupportTacticScorer
{
   public:
    explicit DuplicationScorer();

    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;

    /**
     * The javadoc comment for all `update` methods below can be read as:
     * Records a usage of the support tactic associated with the candidate
     *
     * @param candidate the SupportTacticCandidate whose associated support tactic was used 
     */
    void update(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;

   private:
    // Maps types of tactics to their number of recorded usages
    TypeMap<int> usage_counter_;
};