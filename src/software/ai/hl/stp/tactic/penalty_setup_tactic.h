#pragma once

#include "software/ai/hl/stp/tactic/move/move_tactic.h"

/**
 * The PenaltySetupTactic will move the assigned robot to the given destination and
 * arrive with the specified final orientation and speed in preparation for penalty shot,
 * so it will not avoid the enemy defense area
 */
class PenaltySetupTactic : public MoveTactic
{
   public:
    /**
     * Creates a new PenaltySetupTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit PenaltySetupTactic(bool loop_forever);

    void updateWorldParams(const World& world) override;

    PenaltySetupTactic() = delete;

    void accept(TacticVisitor& visitor) const override;
};
