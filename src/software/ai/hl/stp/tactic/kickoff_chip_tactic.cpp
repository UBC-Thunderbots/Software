#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"

KickoffChipTactic::KickoffChipTactic(bool loop_forever) : ChipTactic(loop_forever) {}

void KickoffChipTactic::updateWorldParams(const World& world) {}

void KickoffChipTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
