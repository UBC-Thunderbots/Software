#include "software/ai/hl/stp/play/dynamic_plays/support_tactic_scorers/success_scorer.h"

double SuccessScorer::score(const SupportTacticCandidate<PassReceiverTactic> &candidate)
{
    return 1;
}

double SuccessScorer::score(const SupportTacticCandidate<FakePassReceiverTactic> &candidate)
{
    return 1;
}

double SuccessScorer::score(const SupportTacticCandidate<CherryPickerTactic> &candidate)
{
    return 1;
}

double SuccessScorer::score(const SupportTacticCandidate<DisrupterTactic> &candidate)
{
    return 1;
}
