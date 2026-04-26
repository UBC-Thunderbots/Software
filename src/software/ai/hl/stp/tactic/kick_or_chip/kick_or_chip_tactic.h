#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick_or_chip/kick_or_chip_fsm.h"
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
     * @param auto_chip_or_kick The mode of how the robot will chip or kick the ball, and
     * the associated parameter for that model
     */
    void updateControlParams(const Point& kick_or_chip_origin,
                             const Angle& kick_or_chip_direction,
                             AutoChipOrKick auto_chip_or_kick);
    /**
     * Updates the control parameters for this KickTactic.
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_or_chip_target The target location where the kick or chip will aim for
     * @param auto_chip_or_kick The mode of how the robot will chip or kick the ball, and
     * the associated parameter for that model
     */
    void updateControlParams(const Point& kick_or_chip_origin,
                             const Point& kickd_or_chip_target,
                             AutoChipOrKick auto_chip_or_kick);

    void accept(TacticVisitor& visitor) const override;
};

// Creates a new tactic called KickoffChipTactic that is a duplicate of ChipTactic
COPY_TACTIC(KickoffChipTactic, KickOrChipTactic)
