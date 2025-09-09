
#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"

#include <algorithm>
#include <iterator>

HaltPlayFSM::HaltPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<HaltPlayFSM>(ai_config_ptr), halt_tactics({{}})
{
    std::generate_n(std::back_inserter(halt_tactics.front()), MAX_ROBOT_IDS_PER_SIDE,
                    [ai_config_ptr]()
                    { return std::make_shared<HaltTactic>(ai_config_ptr); });
}

void HaltPlayFSM::updateStop(const Update& event)
{
    event.common.set_tactics(halt_tactics);
}
