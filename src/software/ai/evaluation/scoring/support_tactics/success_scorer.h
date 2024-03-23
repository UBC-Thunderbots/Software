#pragma once

#include "software/ai/evaluation/scoring/support_tactics/support_tactic_candidate.hpp"
#include "software/ai/evaluation/scoring/support_tactics/support_tactic_scorer.h"
#include "software/util/type_map/type_map.h"

/**
 * Scores SupportTacticCandidates based on past performance and success of
 * each support tactic
 */
class SuccessScorer : public SupportTacticScorer
{
   public:
    explicit SuccessScorer();

    /**
     * Updates the internal SuccessScorer scores of the tactics currently marked as used
     *
     * @param score score between [-1, 1] rating the success of the tactics currently in
     * use
     */
    void evaluate(double score);

    /**
     * The javadoc comment for all `score` methods below can be read as:
     * Visits the given SupportTacticCandidate to score it
     *
     * @param candidate the SupportTacticCandidate to score
     *
     * @return the score for the SupportTacticCandidate, in the range [-1.0, 1.0]
     */
    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;

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
    // Maps types of tactics to a score rating the past performance and success of the
    // tactic
    TypeMap<double> tactic_scores_;

    // Maps types of tactics to their number of recorded usages
    TypeMap<unsigned int> usage_counter_;
};
