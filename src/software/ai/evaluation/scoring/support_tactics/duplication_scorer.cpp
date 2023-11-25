#include "software/ai/evaluation/scoring/support_tactics/duplication_scorer.h"

DuplicationScorer::DuplicationScorer() 
{
}

void DuplicationScorer::recordCandidateSelection(const SupportTacticCandidate &candidate)
{
    //selection_counter_[candidate]++;
}

double DuplicationScorer::score(const TypedSupportTacticCandidate<ReceiverTactic> &candidate)
{
    //int num_selections = selection_counter_[candidate];
    //return std::exp(-num_selections);
    return 1;
}
