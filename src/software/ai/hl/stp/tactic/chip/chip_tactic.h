#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ChipTactic will move the assigned robot to the given chip origin and then
 * chip the ball to the chip target.
 */

class ChipTactic : public Tactic<ChipFSM>
{
   public:
    /**
     * Creates a new ChipTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ChipTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<ChipFSM>> fsm_init() override;
};

// Creates a new tactic called KickoffChipTactic that is a duplicate of ChipTactic
COPY_TACTIC(KickoffChipTactic, ChipTactic)
