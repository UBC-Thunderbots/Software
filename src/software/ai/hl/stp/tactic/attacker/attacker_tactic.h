#pragma once

#include "software/ai/hl/stp/tactic/attacker/attacker_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"

/**
 * This tactic is for a robot performing a pass. It should be used in conjunction with
 * the `ReceiverTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to take the pass as fast as possible
 */
class AttackerTactic : public Tactic<AttackerFSM>
{
   public:
    /**
     * Creates a new AttackerTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit AttackerTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;
   private:
    std::unique_ptr<FSM<AttackerFSM>> fsm_init() override;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    /**
     * Log visualize the control parameters for this AttackerTactic
     *
     * @param world Current state of the world
     * @param control_params The control parameters to visualize
     */
    void visualizeControlParams(const World& world);
};
