#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

#include <algorithm>

PenaltySetupTactic::PenaltySetupTactic(bool loop_forever) : MoveTactic(loop_forever) {}

void PenaltySetupTactic::updateWorldParams(const World& world) {}

void PenaltySetupTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
