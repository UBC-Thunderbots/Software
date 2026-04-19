#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"

/**
 * The KickTactic will move the assigned robot to the given kick origin and then
 * kick the ball to the kick target.
 */

class KickTactic : public TacticBase<KickFSM, GetBehindBallFSM>
{
   public:
    /**
     * Creates a new KickTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KickTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Point& kick_origin, const Angle& kick_direction,
                             double kick_speed_meters_per_second);

    /**
     * Updates the control parameters for this KickTactic.
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Point& kick_origin, const Point& kick_target,
                             double kick_speed_meters_per_second);

    void accept(TacticVisitor& visitor) const override;
};
