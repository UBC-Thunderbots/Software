#pragma once

#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"
#include "software/ai/hl/stp/tactic/support_tactic.h"
#include "software/ai/passing/pass.h"

/**
 * ReceiverTactic is a SupportTactic responsible for receiving passes.
 * It will move to the given receiving position and intercept any
 * kicks or chips directed towards it.
 *
 * Note that this tactic does not take into account the time the pass should occur at;
 * it simply tries to move to the receiving position to receive a potential pass as
 * soon as possible
 */
class ReceiverTactic : public SupportTactic
{
   public:
    /**
     * Creates a new ReceiverTactic
     *
     * @param strategy the Strategy shared by all of AI
     */
    explicit ReceiverTactic(std::shared_ptr<Strategy> strategy);

    void accept(TacticVisitor& visitor) const override;

    void updateReceivingPosition(std::optional<Point> receiving_position) override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param receiving_position The point at which to receive the pass
     * @param enable_one_touch_shot If set to false, the receiver will not perform
     * one-touch shots. The robot will simply receive and dribble.
     */
    void updateControlParams(std::optional<Point> receiving_position,
                             bool enable_one_touch_shot = true);

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::shared_ptr<Strategy> strategy_;

    std::map<RobotId, std::unique_ptr<FSM<ReceiverFSM>>> fsm_map;
    ReceiverFSM::ControlParams control_params;
    TbotsProto::ReceiverTacticConfig receiver_config;
};
