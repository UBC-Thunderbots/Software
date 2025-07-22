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

    void accept(TacticVisitor& visitor) const override;
};
