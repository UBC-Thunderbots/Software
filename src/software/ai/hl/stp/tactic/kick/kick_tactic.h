#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The KickTactic will move the assigned robot to the given kick origin and then
 * kick the ball to the kick target.
 */

class KickTactic : public Tactic<KickFSM>
{
   public:
    /**
     * Creates a new KickTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KickTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<KickFSM>> fsm_init() override;
};
