#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/feasibility_scorer.h"

double FeasibilityScorer::score(const SupportTacticCandidate<PassReceiverTactic> &candidate)
{
    return 1;
}

double FeasibilityScorer::score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate)
{
    return 1;
}

double FeasibilityScorer::score(const SupportTacticCandidate<CherryPickerTactic> &candidate)
{
    return 1;
}

double FeasibilityScorer::score(const SupportTacticCandidate<DisrupterTactic> &candidate)
{
    return 1;
}
