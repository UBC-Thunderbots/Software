#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/tactic/mutable_tactic_visitor.h"
#include "software/ai/hl/stp/tactic/non_mutable_tactic_visitor.h"

KickoffChipTactic::KickoffChipTactic(const Ball& ball, bool loop_forever)
    : ChipTactic(ball, loop_forever)
{
}

std::string KickoffChipTactic::getName() const
{
    return "Kickoff Chip Tactic";
}

void KickoffChipTactic::accept(const NonMutableTacticVisitor& visitor) const
{
    visitor.visit(*this);
}

void KickoffChipTactic::accept(MutableTacticVisitor& visitor)
{
    visitor.visit(*this);
}
