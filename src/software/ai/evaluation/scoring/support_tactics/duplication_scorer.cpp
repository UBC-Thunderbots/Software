#include "software/ai/evaluation/scoring/support_tactics/duplication_scorer.h"

DuplicationScorer::DuplicationScorer() : usage_counter_() {}

double DuplicationScorer::score(
    const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    int num_selections = static_cast<int>(usage_counter_.at<ReceiverTactic>());
    return std::exp(-num_selections);
}

void DuplicationScorer::update(
    const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    ++usage_counter_.at<ReceiverTactic>();
}

void DuplicationScorer::reset()
{
    usage_counter_.clear();
}