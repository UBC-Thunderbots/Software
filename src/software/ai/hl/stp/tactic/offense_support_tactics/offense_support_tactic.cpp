#include "software/ai/hl/stp/tactic/offense_support_tactics/offense_support_tactic.h"

OffenseSupportTactic::OffenseSupportTactic(
    const std::set<RobotCapability> &capability_reqs, std::shared_ptr<Strategy> strategy)
    : Tactic(capability_reqs),
      strategy_(strategy)
{
}

void OffenseSupportTactic::commit()
{
    (*strategy_)->commit(getOffenseSupportType());
}
