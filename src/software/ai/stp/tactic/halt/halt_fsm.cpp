#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"

HaltFSM::HaltFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : TacticFSM<HaltFSM>(ai_config_ptr)
{
}

void HaltFSM::updateStop(const Update& event)
{
    event.common.set_primitive(std::make_unique<StopPrimitive>());
}

bool HaltFSM::stopDone(const Update& event)
{
    return robotStopped(event.common.robot);
}
