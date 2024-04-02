#pragma once

#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

/**
 * This tactic is for a robot receiving a pass. It should be used in conjunction with
 * the `AttackerTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to receive the pass as possible
 */
class ReceiverTactic : public Tactic
{
   public:
    ReceiverTactic(std::shared_ptr<Strategy> strategy);

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param updated_pass The pass this tactic should try to receive
     * @param disable_one_touch_shot If set to true, the receiver will not perform a
     * one-touch The robot will simply receive and dribble.
     */
    void updateControlParams(std::optional<Pass> updated_pass,
                             bool disable_one_touch_shot = false);

    void prepare() override;

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::shared_ptr<Strategy> strategy_;

    std::map<RobotId, std::unique_ptr<FSM<ReceiverFSM>>> fsm_map;
    ReceiverFSM::ControlParams control_params;
};
