#include "software/ai/hl/stp/tactic/tactic.h"

#include "software/ai/intent/stop_intent.h"
#include "software/logger/logger.h"

Tactic::Tactic(bool loop_forever, const std::set<RobotCapability> &capability_reqs_)
    : loop_forever(loop_forever), capability_reqs(capability_reqs_)
{
}

bool Tactic::isGoalieTactic() const
{
    // TODO (#1859): Delete this!
    return false;
}

const std::set<RobotCapability> &Tactic::robotCapabilityRequirements() const
{
    return capability_reqs;
}

std::set<RobotCapability> &Tactic::mutableRobotCapabilityRequirements()
{
    return capability_reqs;
}

std::unique_ptr<Intent> Tactic::next(const Robot &robot, const World &world)
{
    updateFSM(TacticFSMUpdate{.robot      = robot,
                              .world      = world,
                              .set_intent = [this](std::unique_ptr<Intent> new_intent) {
                                  intent = std::move(new_intent);
                              }});
    if (intent)
    {
        return std::move(intent);
    }
    else
    {
        LOG(WARNING) << "No intent set for this tactic" << std::endl;
        return std::make_unique<StopIntent>(robot.id(), false);
    }
}
