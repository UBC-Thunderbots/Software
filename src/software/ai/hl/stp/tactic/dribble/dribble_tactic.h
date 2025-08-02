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

    /**
     * Updates control params for optionally moving the ball to a dribble destination and
     * with the robot at a final dribble orientation
     *
     * @param dribble_destination The destination for dribbling the ball
     * @param final_dribble_orientation The final orientation to face the ball when
     * finishing dribbling
     * @param allow_excessive_dribbling Whether to allow excessive dribbling, i.e. more
     * than 1 metre at a time
     */
    void updateControlParams(std::optional<Point> dribble_destination,
                             std::optional<Angle> final_dribble_orientation,
                             bool allow_excessive_dribbling = false);

    void accept(TacticVisitor& visitor) const override;
};

COPY_TACTIC(PlaceBallTactic, DribbleTactic)
