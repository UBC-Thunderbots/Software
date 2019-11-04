#include "software/ai/hl/stp/tactic/kickoff_chip_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/action/chip_action.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"

KickoffChipTactic::KickoffChipTactic(const Ball &ball, bool loop_forever)
    : ChipTactic(ball, loop_forever)
{
    // the chipper is allowed to go into the centre circle and touch the ball
    addWhitelistedAvoidArea(AvoidArea::CENTER_CIRCLE);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);

    std::cout << this->getName() << std::endl;
}

std::string KickoffChipTactic::getName() const
{
    return "Kickoff Chip Tactic";
}

void KickoffChipTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
