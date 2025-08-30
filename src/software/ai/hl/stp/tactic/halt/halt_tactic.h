#pragma once

#include "software/ai/hl/stp/tactic/halt/halt_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.hpp"

/**
 * The HaltTactic will stop the robot from moving. The robot will actively try and brake
 * to come to a halt.
 */
class HaltTactic : public TacticBase<HaltFSM>
{
   public:
    /**
     * Creates a new HaltTactic
     */
    explicit HaltTactic(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr);

    void accept(TacticVisitor& visitor) const override;
};
