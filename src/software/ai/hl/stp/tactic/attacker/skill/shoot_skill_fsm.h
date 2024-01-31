#pragma once

#include "software/ai/hl/stp/tactic/attacker/attacker_fsm.h"
#include "software/ai/hl/stp/tactic/attacker/skill/skill_fsm.h"

class ShootSkillFSM : public SkillFSM
{
   public:
    explicit ShootSkillFSM(std::shared_ptr<Strategy> strategy)
        : SkillFSM(strategy),
          fsm(DribbleFSM(strategy->getAiConfig().dribble_tactic_config()),
              AttackerFSM(strategy->getAiConfig().attacker_tactic_config())),
          control_params({.best_pass_so_far = std::nullopt,
                          .pass_committed   = false,
                          .shot             = std::nullopt,
                          .chip_target      = std::nullopt})
    {
    }

    void updatePrimitive(const TacticUpdate& tactic_update) override;
    bool done() const override;

   private:
    FSM<AttackerFSM> fsm;
    AttackerFSM::ControlParams control_params;
};
