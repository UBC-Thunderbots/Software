#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The GetBehindBallTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed
 */
class GetBehindBallTactic : public Tactic<GetBehindBallFSM>
{
   public:
    /**
     * Creates a new GetBehindBallTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit GetBehindBallTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor& visitor) const override;
};
