#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"

/**
 * The KickOrChipTactic will move the assigned robot to the given kick origin and then
 * kick or chip the ball depending on the control params passed in.
 */

class KickOrChipTactic : public TacticBase<KickOrChipFSM, GetBehindBallFSM>
{
   public:
    /**
     * Creates a new KickOrChipTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit KickOrChipTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param kick_or_chip_origin The location where the kick will be taken
     * @param kick_or_chip_direction The direction the Robot will kick in
     * @param isChipping Whether the ball should be chipped
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     * @param chip_distance_meters The distance of how far we want to chip the ball
     */
    void updateControlParams(const Point& kick_or_chip_origin, const Angle& kick_or_chip_direction,
			     bool isChipping, double kick_speed_meters_per_second,
			     double chip_distance_meters);
    /**
     * Updates the control parameters for this KickTactic.
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_or_chip_target The target location where the kick or chip will aim for
     * @param isChipping Whether the ball should be chipped
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     * @param chip_distance_meters The distance of how far we want to chip the ball
     */
    void updateControlParams(const Point& kick_or_chip_origin, const Angle& kick_or_chip_target,
			     bool isChipping, double kick_speed_meters_per_second);

    void accept(TacticVisitor& visitor) const override;
};
