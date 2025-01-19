
#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"

#include <algorithm>
#include <iterator>

HaltPlayFSM::HaltPlayFSM(TbotsProto::AiConfig ai_config) : halt_tactics({{}})
{
    std::generate_n(std::back_inserter(halt_tactics.front()), MAX_ROBOT_IDS_PER_SIDE,
                    []() { return std::make_shared<HaltTactic>(); });
}

void HaltPlayFSM::updateStop(const Update& event)
{
    event.common.set_tactics(halt_tactics);
}
