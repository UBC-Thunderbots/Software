#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"

void HaltFSM::updateStop(const Update& event)
{
    event.common.set_primitive(std::make_unique<StopPrimitive>());
}

bool HaltFSM::stopDone(const Update& event)
{
    return robotStopped(event.common.robot);
}
