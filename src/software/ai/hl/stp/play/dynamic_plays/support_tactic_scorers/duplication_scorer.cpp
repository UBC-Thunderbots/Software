#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/duplication_scorer.h"

DuplicationScorer::DuplicationScorer() : selection_counter_()
{
}

void DuplicationScorer::recordCandidateSelection(const SupportTacticCandidate<Tactic> &candidate)
{
    selection_counter_[candidate]++;
}

double DuplicationScorer::score(const SupportTacticCandidate<PassReceiverTactic> &candidate)
{
    int num_selections = selection_counter_[candidate];
    return std::exp(-num_selections);
}

double DuplicationScorer::score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate)
{
    int num_selections = selection_counter_[candidate];
    return std::exp(-num_selections);
}

double DuplicationScorer::score(const SupportTacticCandidate<CherryPickerTactic> &candidate)
{
    int num_selections = selection_counter_[candidate];
    return std::exp(-num_selections);
}

double DuplicationScorer::score(const SupportTacticCandidate<DisrupterTactic> &candidate)
{
    int num_selections = selection_counter_[candidate];
    return std::exp(-num_selections);
}
