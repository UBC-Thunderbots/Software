#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"

KickoffChipTactic::KickoffChipTactic(const Ball& ball, bool loop_forever)
    : ChipTactic(ball, loop_forever)
{
}

std::string KickoffChipTactic::getName() const
{
    return "Kickoff Chip Tactic";
}

void KickoffChipTactic::accept(MutableTacticVisitor& visitor)
{
    visitor.visit(*this);
}
