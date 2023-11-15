#pragma once

#include <set>

#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/support_tactic_scorer.h"
#include "software/ai/hl/stp/tactic/pass_receiver/pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/fake_pass_receiver/fake_pass_receiver_tactic.h"
#include "software/ai/hl/stp/tactic/cherry_picker/cherry_picker_tactic.h"
#include "software/ai/hl/stp/tactic/disrupter/disrupter_tactic.h"

/**
 * A SupportTacticCandidate is a "type wrapper" that makes a type of tactic
 * eligible for scoring and assignment as a support tactic in a DynamicPlay.
 *
 * It acts as the visitee for a SupportTacticScorer, which operates under
 * the visitor pattern.
 */
template<typename SupportTacticType>
class SupportTacticCandidate 
{
   public:
    explicit SupportTacticCandidate();

    /**
     * Accepts a SupportTacticScorer and calls the score function on itself
     *
     * @param scorer a SupportTacticScorer
     */
    void score(SupportTacticScorer &scorer);

    /**
     * Resets the total score for this support tactic candidate
     */
    void resetTotalScore();

    /**
     * Gets the total score for this support tactic candidate
     *
     * @return the total score for this support tactic candidate
     */
    double getTotalScore() const;

    /**
     * Returns a shared pointer to a newly constructed instance of the
     * support tactic type
     *
     * @return a shared pointer to a newly constructed instance of the given
     * support tactic type
     */
    std::shared_ptr<SupportTacticType> createSupportTactic() const;

   private:
    double total_score_;
}

template<typename SupportTacticType>
SupportTacticCandidate<SupportTacticType>::SupportTacticCandidate() : score_(1) 
{
}

template<typename SupportTacticType>
void SupportTacticCandidate<SupportTacticType>::score(SupportTacticScorer &scorer)
{
    total_score_ *= scorer.score(*this);
}

template<typename SupportTacticType>
void SupportTacticCandidate<SupportTacticType>::resetTotalScore()
{
    total_score_ = 1;
}

template<typename SupportTacticType>
double SupportTacticCandidate<SupportTacticType>::getTotalScore() const
{
    return total_score_;
}

template<typename SupportTacticType>
std::shared_ptr<SupportTacticType> SupportTacticCandidate<SupportTacticType>::create() const
{
    return std::make_shared<SupportTacticType>();
}

using SupportTacticCandidateVector = std::vector<std::shared_ptr<SupportTacticCandidate<Tactic>>>;

/**
 * Returns all support tactic candidates
 * 
 * @return a vector of shared pointers to all support tactic candidates
 */
static SupportTacticCandidateVector allSupportTacticCandidates() 
{
    return 
    {
        std::make_shared<SupportTacticCandidate<PassReceiverTactic>>(),
        std::make_shared<SupportTacticCandidate<FakePassReceiverTactic>>(),
        std::make_shared<SupportTacticCandidate<CherryPickerTactic>>(),
        std::make_shared<SupportTacticCandidate<DisrupterTactic>>()
    };
}