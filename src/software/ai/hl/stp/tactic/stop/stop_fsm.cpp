#include "software/ai/hl/stp/tactic/stop/stop_fsm.h"

void StopFSM::updateStop(const Update& event)
{
    event.common.set_intent(std::make_unique<StopIntent>(event.common.robot.id(), coast));
}
