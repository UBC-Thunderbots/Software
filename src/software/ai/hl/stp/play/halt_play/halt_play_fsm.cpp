//
// Created by grayson on 2024-10-19.
//

#include "software/ai/hl/stp/play/halt_play/halt_play_fsm.h"

void HaltPlayFSM::updateStop(const Update& event)
{
    halt_tactics = {{std::make_shared<StopTactic>(), std::make_shared<StopTactic>(),
                     std::make_shared<StopTactic>(), std::make_shared<StopTactic>(),
                     std::make_shared<StopTactic>(), std::make_shared<StopTactic>()}};
    event.common.set_tactics(halt_tactics);
}
