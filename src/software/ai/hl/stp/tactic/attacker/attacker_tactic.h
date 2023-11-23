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
class AttackerTactic : public Tactic
{
   public:
    /**
     * Creates a new AttackerTactic
     *
     * @param ai_config The AI configuration
     */
    explicit AttackerTactic(TbotsProto::AiConfig ai_config);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

    void setLastExecutionRobot(std::optional<RobotId> last_execution_robot) override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    // AI config
    TbotsProto::AiConfig ai_config;

    std::stack<Skill> skill_sequence;

    std::map<RobotId, std::shared_ptr<Skill>> next_skill_map;
};
