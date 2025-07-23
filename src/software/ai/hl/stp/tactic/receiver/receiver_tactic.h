#pragma once

#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/ray.h"

/**
 * This tactic is for a robot receiving a pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to receive the pass as possible
 */
class ReceiverTactic : public Tactic<ReceiverFSM>
{
   public:
    /**
     * Creates a new ReceiverTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ReceiverTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param updated_pass The pass this tactic should try to receive
     * @param disable_one_touch_shot If set to true, the receiver will not perform a
     * one-touch The robot will simply receive and dribble.
     */
    void updateControlParams(std::optional<Pass> updated_pass,
                             bool disable_one_touch_shot = false);

    void accept(TacticVisitor& visitor) const override;


   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    ReceiverFSMControlParams control_params;
};
