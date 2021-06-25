#pragma once

#include "software/ai/hl/stp/tactic/move/move_tactic.h"

/**
 * The KickoffMoveTactic will move the assigned robot to the given origin and orientation. The robot assigned this tactic is allowed in the
 * centre circle and near the ball during kickoff.
 */

class KickoffMoveTactic : public MoveTactic
{
   public:
    /**
     * Creates a new KickoffMoveTactic
     *
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit KickoffMoveTactic(bool loop_forever);

    KickoffMoveTactic() = delete;

    void updateWorldParams(const World& world) override;

    void accept(TacticVisitor& visitor) const override;
};
