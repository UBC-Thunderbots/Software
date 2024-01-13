#include "software/ai/evaluation/scoring/support_tactics/duplication_scorer.h"

DuplicationScorer::DuplicationScorer() {}

double DuplicationScorer::score(
    const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    int num_selections = usage_counter_.getOrDefault<ReceiverTactic>();
    return std::exp(-num_selections);
}

void DuplicationScorer::update(
    const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    int num_selections = usage_counter_.getOrDefault<ReceiverTactic>();
    usage_counter_.put<ReceiverTactic>(num_selections + 1);
}
