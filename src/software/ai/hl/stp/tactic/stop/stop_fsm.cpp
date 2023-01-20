#include "software/ai/hl/stp/tactic/stop/stop_fsm.h"

void StopFSM::updateStop(const Update& event)
{
    event.common.set_primitive(createStopPrimitive(coast));
}

bool StopFSM::stopDone(const Update& event)
{
    return robotStopped(event.common.robot);
}
