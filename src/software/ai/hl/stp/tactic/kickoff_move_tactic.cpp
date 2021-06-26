#include "software/ai/hl/stp/tactic/kickoff_move_tactic.h"

#include <algorithm>

KickoffMoveTactic::KickoffMoveTactic(bool loop_forever) : MoveTactic(loop_forever) {}

void KickoffMoveTactic::updateWorldParams(const World& world) {}

void KickoffMoveTactic::accept(TacticVisitor& visitor) const
{
    visitor.visit(*this);
}
