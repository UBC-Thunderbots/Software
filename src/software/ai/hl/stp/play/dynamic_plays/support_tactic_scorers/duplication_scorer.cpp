#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/duplication_scorer.h"

DuplicationScorer::DuplicationScorer() : tactic_usages_()
{
}

void DuplicationScorer::recordTacticUsage(const SupportTacticCandidate<Tactic> &candidate)
{
    tactic_usages_[candidate]++;
}

double DuplicationScorer::score(const SupportTacticCandidate<PassReceiverTactic> &candidate)
{
    int num_usages = tactic_usages_[candidate];
    return std::exp(-num_usages);
}

double DuplicationScorer::score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate)
{
    int num_usages = tactic_usages_[candidate];
    return std::exp(-num_usages);
}

double DuplicationScorer::score(const SupportTacticCandidate<CherryPickerTactic> &candidate)
{
    int num_usages = tactic_usages_[candidate];
    return std::exp(-num_usages);
}

double DuplicationScorer::score(const SupportTacticCandidate<DisrupterTactic> &candidate)
{
    int num_usages = tactic_usages_[candidate];
    return std::exp(-num_usages);
}
