#pragma once

#include "software/ai/hl/stp/tactic/chip_tactic.h"

/**
 * The KickoffChipTactic will move the assigned robot to the given chip origin and then
 * chip the ball to the chip target. The robot assigned this tactic is allowed in the
 * centre circle and near the ball during kickoff.
 */

class KickoffChipTactic : public ChipTactic
{
   public:
    /**
     * Creates a new KickoffChipTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit KickoffChipTactic(const Ball& ball, bool loop_forever);

    KickoffChipTactic() = delete;

    void updateWorldParams(const World& world) override;

    void accept(TacticVisitor& visitor) const override;
};
