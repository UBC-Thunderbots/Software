#pragma once

#include <stack>

#include "software/ai/hl/stp/tactic/attacker/skill/head_skill.h"
#include "software/ai/hl/stp/tactic/tactic.h"

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
    explicit AttackerTactic(TbotsProto::AiConfig ai_config,
                            std::shared_ptr<Strategy> strategy);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

    inline bool done() const override
    {
        return false;
    }

    std::string getFSMState() const override;

    void setLastExecutionRobot(std::optional<RobotId> last_execution_robot) override;

    void updateAiConfig(const TbotsProto::AiConfig& ai_config);

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    TbotsProto::AiConfig ai_config;  // AI config
    std::shared_ptr<Strategy> strategy;

    std::shared_ptr<HeadSkill> head_skill;
    std::stack<std::shared_ptr<Skill>> skill_sequence;

    std::map<RobotId, std::shared_ptr<Skill>> next_skill_map;
};
