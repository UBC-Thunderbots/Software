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
    explicit SuccessScorer() = default;

    double score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override;

    // Score update methods for each support tactic
    void update(const TypedSupportTacticCandidate<OffenseSupportTactic> &candidate) =
        delete;
    void update(const TypedSupportTacticCandidate<ReceiverTactic> &candidate) override{};

   private:
    TypeMap<double> tactic_scores_;
};
