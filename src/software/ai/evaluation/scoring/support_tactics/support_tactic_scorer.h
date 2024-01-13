#pragma once

#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

// We forward-declare TypedSupportTacticCandidate because if we include it, we induce a
// circular dependency between support_tactic_candidate.h and support_tactic_scorer.hpp.
template <typename TSupportTactic>
class TypedSupportTacticCandidate;

/**
 * Visits SupportTacticCandidates to score them.
 *
 * We use the visitor pattern to ensure all SupportTacticScorers implement
 * a `score` method for every instantiation of TypedSupportTacticCandidate
 * at compile time.
 */
class SupportTacticScorer
{
   public:
    /**
     * The javadoc comment for all `score` methods below can be read as:
     * Visits the given SupportTacticCandidate to score it
     *
     * @param candidate the SupportTacticCandidate to score
     *
     * @return the score for the SupportTacticCandidate, in the range [-1.0, 1.0]
     */
    virtual double score(
        const TypedSupportTacticCandidate<ReceiverTactic> &candidate) = 0;

    /**
     * The javadoc comment for all `update` methods below can be read as:
     * Updates the state of the scorer based on the SupportTacticCandidate it visits
     *
     * @param candidate the SupportTacticCandidate to visit
     */
    virtual void update(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) {}
};
