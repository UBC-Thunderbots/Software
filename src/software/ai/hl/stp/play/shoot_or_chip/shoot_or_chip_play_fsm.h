#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_fsm.hpp"
#include "software/ai/hl/stp/tactic/attacker/attacker_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

struct ShootOrChipPlayFSM : PlayFSM<ShootOrChipPlayFSM>
  {
      struct ControlParams
      {
      };

      class ShootOrChipState;

      explicit ShootOrChipPlayFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

      void updateShootOrChip(const Update& event);

	  bool attackerDone(const Update& event);
	
	

      auto operator()()
      {
          using namespace boost::sml;

          DEFINE_SML_STATE(ShootOrChipState)
          DEFINE_SML_EVENT(Update)
          DEFINE_SML_ACTION(updateShootOrChip)
		  DEFINE_SML_GUARD(attackerDone)

          return make_transition_table(
              *ShootOrChipState_S + Update_E [!attackerDone_G] / updateShootOrChip_A = ShootOrChipState_S,
			  ShootOrChipState_S + Update_E [attackerDone_G] / updateShootOrChip_A = X,
              X + Update_E / updateShootOrChip_A = X);
      }

     private:
      std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics;
      std::array<std::shared_ptr<MoveTactic>, 2> move_to_open_area_tactics;
      std::shared_ptr<AttackerTactic> attacker;
  };
