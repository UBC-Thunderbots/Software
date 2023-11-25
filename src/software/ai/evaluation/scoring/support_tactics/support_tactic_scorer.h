#pragma once

#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

// We forward-declare SupportTacticCandidate because if we include it, we induce a 
// circular dependency between support_tactic_candidate.h and support_tactic_scorer.hpp.
template <typename TSupportTactic>
class TypedSupportTacticCandidate;

/**
 * Visits SupportTacticCandidates to score them.
 *
 * We use the visitor pattern to ensure all SupportTacticScorers implement
 * a `score` method for every SupportTacticCandidate at compile time.
 */
class SupportTacticScorer 
{
   public:
    virtual ~SupportTacticScorer() = default;

    /**
     * The javadoc comment for all `score` methods below can be read as:
     * Visits the given support tactic candidate to score it
     *
     * @param candidate the SupportTacticCandidate to score
     *
     * @return the score for the SupportTacticCandidate, in the range [-1.0, 1.0]
     */
    virtual double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) = 0;
};
