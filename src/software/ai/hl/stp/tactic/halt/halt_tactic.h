#pragma once

#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The HaltTactic will stop the robot from moving. The robot will actively try and brake
 * to come to a halt.
 */
class HaltTactic : public Tactic
{
   public:
    /**
     * Creates a new HALTTactic
     */
    explicit HaltTactic();

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<HaltFSM>>> fsm_map;
};
