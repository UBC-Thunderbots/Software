#include "software/ai/hl/stp/tactic/tactic.h"

#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"
#include "software/util/typename/typename.h"

Tactic::Tactic(bool loop_forever, const std::set<RobotCapability> &capability_reqs_)
    : intent(),
      loop_forever(loop_forever),
      capability_reqs(capability_reqs_)
{
}

const std::set<RobotCapability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

std::unique_ptr<Intent> Tactic::get(const Robot &robot, const World &world)
{
    updateIntent(TacticUpdate(robot, world, [this](std::unique_ptr<Intent> new_intent) {
        intent = std::move(new_intent);
    }));

    if (intent)
    {
        return std::move(intent);
    }
    else
    {
        return std::make_unique<StopIntent>(robot.id(), false);
    }
}
