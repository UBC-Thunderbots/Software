#pragma once

#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The MoveTactic will move the assigned robot to the given destination and arrive
 * with the specified final orientation and speed
 */
class MoveTactic : public Tactic<MoveFSM>
{
   public:
    /**
     * Creates a new MoveTactic
     */
    explicit MoveTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor& visitor) const override;
};

// Creates duplicates of MoveTactic for various situations
COPY_TACTIC(PenaltySetupTactic, MoveTactic)
COPY_TACTIC(MoveGoalieToGoalLineTactic, MoveTactic)
COPY_TACTIC(PrepareKickoffMoveTactic, MoveTactic)
COPY_TACTIC(PlaceBallMoveTactic, MoveTactic)
COPY_TACTIC(AvoidInterferenceTactic, MoveTactic)
