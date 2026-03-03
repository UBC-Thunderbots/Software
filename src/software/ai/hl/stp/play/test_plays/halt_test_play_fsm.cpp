#include "software/ai/hl/stp/play/test_plays/halt_test_play_fsm.h"

HaltTestPlayFSM::HaltTestPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
    : PlayFSM<HaltTestPlayFSM>(ai_config_ptr),
      halt_tactics({{
          std::make_shared<HaltTactic>(ai_config_ptr),
          std::make_shared<HaltTactic>(ai_config_ptr),
          std::make_shared<HaltTactic>(ai_config_ptr),
      }})
{
}

void HaltTestPlayFSM::updateHalt(const Update& event)
{
    event.common.set_tactics(halt_tactics);
}
