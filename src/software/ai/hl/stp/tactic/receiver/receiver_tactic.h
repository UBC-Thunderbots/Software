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
    explicit ReceiverTactic(std::shared_ptr<Strategy> strategy);

    void accept(TacticVisitor& visitor) const override;

    std::map<RobotId, std::shared_ptr<Primitive>> get(const WorldPtr& world_ptr) override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::shared_ptr<Strategy> strategy_;

    std::map<RobotId, std::unique_ptr<FSM<ReceiverFSM>>> fsm_map;
    ReceiverFSM::ControlParams control_params;
};
