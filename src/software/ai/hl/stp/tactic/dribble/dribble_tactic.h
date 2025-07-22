#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The DribbleTactic will move the robot to intercept the ball and optionally dribble it
 * to the dribble destination with the robot facing the given direction.
 * It also optionally applies small kicks to not excessively dribble
 *
 * Done: When the ball is near the dribbler of the robot and the optional dribble
 * destination and face ball orientation conditions are satisfied
 */
class DribbleTactic : public Tactic<DribbleFSM>
{
   public:
    /**
     * Creates a new DribbleTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit DribbleTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    DribbleTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

};

COPY_TACTIC(PlaceBallTactic, DribbleTactic)
