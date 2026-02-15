#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/halt/halt_tactic.h"

struct HaltTestPlayFSM : PlayFSM<HaltTestPlayFSM>
  {
      struct ControlParams
      {
      };

      class HaltTestState;

      explicit HaltTestPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
          : PlayFSM<HaltTestPlayFSM>(ai_config_ptr),
            halt_tactics({{
                std::make_shared<HaltTactic>(ai_config_ptr),
                std::make_shared<HaltTactic>(ai_config_ptr),
                std::make_shared<HaltTactic>(ai_config_ptr),
            }})
      {
      }

      void updateHalt(const Update& event)
      {
          event.common.set_tactics(halt_tactics);
      }

      auto operator()()
      {
          using namespace boost::sml;

          DEFINE_SML_STATE(HaltTestState)
          DEFINE_SML_EVENT(Update)
          DEFINE_SML_ACTION(updateHalt)

          return make_transition_table(
              *HaltTestState_S + Update_E / updateHalt_A = HaltTestState_S,
              X + Update_E / updateHalt_A = X);
      }

     private:
      PriorityTacticVector halt_tactics;
  };
