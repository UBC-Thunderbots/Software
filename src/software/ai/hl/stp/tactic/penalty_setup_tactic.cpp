#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

#include <algorithm>

PenaltySetupTactic::PenaltySetupTactic(bool loop_forever) : MoveTactic(loop_forever) {}

std::string PenaltySetupTactic::getName() const
{
    return "Penalty Setup Tactic";
}

void PenaltySetupTactic::accept(MutableTacticVisitor& visitor)
{
    visitor.visit(*this);
}
