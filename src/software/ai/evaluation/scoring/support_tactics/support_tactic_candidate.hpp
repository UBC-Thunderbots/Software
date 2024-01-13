#pragma once

#include <set>

#include "software/ai/evaluation/scoring/candidate.h"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"

/**
 * A SupportTacticCandidate makes a type of tactic eligible for scoring and
 * assignment as a support tactic in a DynamicPlay.
 *
 * It acts as the visitee for a SupportTacticScorer, which operates under
 * the visitor pattern.
 */
class SupportTacticCandidate : public Candidate
{
   public:
    /**
     * Accepts a SupportTacticScorer and calls the scorer's `score` function
     * on itself. The scorer's returned score is applied to this candidate.
     *
     * @param scorer a SupportTacticScorer to use to score this candidate
     */
    virtual void score(SupportTacticScorer &scorer) = 0;

    /**
     * Accepts a SupportTacticScorer and calls the scorer's `update` function
     * on itself. This lets SupportTacticScorer update its state based on
     * this SupportTacticCandidate
     *
     * @param scorer a SupportTacticScorer to update
     */
    virtual void updateScorer(SupportTacticScorer &scorer) = 0;

    /**
     * Returns a shared pointer to a newly constructed instance of the
     * type of support tactic this candidate represents
     *
     * @return a shared pointer to a newly constructed instance of the
     * type of support tactic this candidate represents
     */
    virtual std::shared_ptr<Tactic> createSupportTactic() = 0;

   protected:
    explicit SupportTacticCandidate() = default;
};

/**
 * This class template lets us generate subtypes of SupportTacticCandidate associated
 * with a type of tactic.
 *
 * @tparam TSupportTactic the type of support tactic that this candidate represents
 */
template <typename TSupportTactic>
class TypedSupportTacticCandidate : public SupportTacticCandidate
{
    static_assert(std::is_base_of<Tactic, TSupportTactic>::value,
                  "TSupportTactic must derive from Tactic");

   public:
    explicit TypedSupportTacticCandidate() = default;

    void score(SupportTacticScorer &scorer) override
    {
        applyScore(scorer.score(*this));
    }

    void updateScorer(SupportTacticScorer &scorer) override
    {
        scorer.update(*this);
    }

    std::shared_ptr<Tactic> createSupportTactic() override
    {
        return std::make_shared<TSupportTactic>();
    }
};
