#pragma once

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"
#include "software/util/type_map/type_map.h"

/**
 * Scores SupportTacticCandidates based on how many times they have already been selected
 */
class DuplicationScorer : public SupportTacticScorer
{
   public:
    explicit DuplicationScorer();
 
    /**
     * The javadoc comment for all `score` methods below can be read as:
     * Visits the given SupportTacticCandidate to score it
     *
     * @param candidate the SupportTacticCandidate to score
     *
     * @return the score for the SupportTacticCandidate, in the range [-1.0, 1.0]
     */
    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate);

    /**
     * The javadoc comment for all `update` methods below can be read as:
     * Records a usage of the support tactic associated with the candidate
     *
     * @param candidate the SupportTacticCandidate whose associated support tactic was
     * used
     */
    void update(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;
    
    /**
     * Marks all support tactics as unused
     */
    void reset() override;
   
   private:
    // Maps types of tactics to their number of recorded usages
    TypeMap<unsigned int> usage_counter_;
};
