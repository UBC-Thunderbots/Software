#pragma once

#include "software/ai/hl/stp/tactic/get_behind_ball/get_behind_ball_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.h"

/**
 * The GetBehindBallTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed
 */
class GetBehindBallTactic : public TacticBase<GetBehindBallFSM>
{
   public:
    /**
     * Creates a new GetBehindBallTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit GetBehindBallTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the control parameters for this GetBehindBallTactic.
     *
     * @param ball_location The location of the ball when it will be chipped or kicked
     * @param chick_direction The direction to kick or chip
     */
    void updateControlParams(const Point& ball_location, Angle chick_direction);

    void accept(TacticVisitor& visitor) const override;
};
