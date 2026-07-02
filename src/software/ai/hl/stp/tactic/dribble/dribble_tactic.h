#pragma once

#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"

/**
 * The DribbleTactic will move the robot to intercept the ball and optionally dribble it
 * to the dribble destination with the robot facing the given direction.
 * It also optionally applies small kicks to not excessively dribble
 *
 * Done: When the ball is near the dribbler of the robot and the optional dribble
 * destination and face ball orientation conditions are satisfied
 */
class DribbleTactic : public TacticBase<DribbleFSM>
{
   public:
    /**
     * Creates a new DribbleTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit DribbleTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

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
     * @param max_speed_dribble The max speed allowed while we are dribbling the ball
     * @param max_speed_get_possession The max speed allowed while we are moving towards
     * the ball to get possession
     */
    void updateControlParams(std::optional<Point> dribble_destination,
                             std::optional<Angle> final_dribble_orientation,
                             bool allow_excessive_dribbling = false,
                             TbotsProto::MaxAllowedSpeedMode max_speed_dribble =
                                 TbotsProto::MaxAllowedSpeedMode::DRIBBLE,
                             TbotsProto::MaxAllowedSpeedMode max_speed_get_possession =
                                 TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT);

    void accept(TacticVisitor& visitor) const override;
};

COPY_TACTIC(BallPlacementDribbleTactic, DribbleTactic)
