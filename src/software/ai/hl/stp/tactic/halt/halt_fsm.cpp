#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"

void HaltFSM::updateHalt(const Update& event)
{
    event.common.set_primitive(createStopPrimitiveProto());
}

bool HaltFSM::haltDone(const Update& event)
{
    return robotStopped(event.common.robot);
}
