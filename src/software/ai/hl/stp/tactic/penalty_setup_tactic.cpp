#include "software/ai/hl/stp/tactic/penalty_setup_tactic.h"

#include <algorithm>

#include "software/ai/hl/stp/tactic/tactic_visitor.h"

PenaltySetupTactic::PenaltySetupTactic(bool loop_forever) : MoveTactic(loop_forever)
{
    addWhitelistedAvoidArea(AvoidArea::ENEMY_HALF);
    addWhitelistedAvoidArea(AvoidArea::ENEMY_DEFENSE_AREA);
    addWhitelistedAvoidArea(AvoidArea::FRIENDLY_HALF);
    addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
}

std::string PenaltySetupTactic::getName() const
{
    return "Penalty Setup Tactic";
}

void PenaltySetupTactic::accept(TacticVisitor &visitor) const
{
    visitor.visit(*this);
}
