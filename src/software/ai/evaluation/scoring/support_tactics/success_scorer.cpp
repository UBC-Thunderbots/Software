#include "software/ai/evaluation/scoring/support_tactics/success_scorer.h"

SuccessScorer::SuccessScorer() : tactic_scores_(), usage_counter_() {}

void SuccessScorer::evaluate(double score)
{
}

double SuccessScorer::score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    return tactic_scores_.at<ReceiverTactic>();
}

void SuccessScorer::update(const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    ++usage_counter_.at<ReceiverTactic>();
}

void SuccessScorer::reset()
{
    usage_counter_.clear();
}