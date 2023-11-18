#pragma once

#include "software/ai/hl/stp/tactic/pass_receiver/pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/fake_pass_receiver/fake_pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/cherry_picker/cherry_picker_tactic.h"
#include "software/ai/hl/stp/tactic/disrupter/disrupter_tactic.h"

// We forward-declare SupportTacticCandidate because if we include it, we induce a 
// circular dependency between support_tactic_candidate.h and support_tactic_scorer.hpp.
class SupportTacticCandidate;

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
     * @param tactic The support tactic candidate to score
     *
     * @return the score for the support tactic candidate
     */

    virtual double score(const SupportTacticCandidate<PassReceiverTactic> &candidate)       = 0;
    virtual double score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate)   = 0;
    virtual double score(const SupportTacticCandidate<CherryPickerTactic> &candidate)       = 0;
    virtual double score(const SupportTacticCandidate<DisrupterTactic> &candidate)          = 0;
}   
