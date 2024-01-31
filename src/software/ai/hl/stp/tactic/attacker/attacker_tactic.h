#pragma once

#include <stack>

#include "software/ai/evaluation/scoring/skills/skill_graph.h"
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

    /**
     * Updates the control parameters for this AttackerTactic.
     *
     * @param updated_pass The pass to perform
     */
    void updateControlParams(const Pass& best_pass_so_far, bool pass_committed);

    /**
     * Updates the control parameters for this AttackerTactic
     *
     * @param chip_target An optional point that the robot will chip towards when it is
     * unable to shoot and is in danger of losing the ball to an enemy. If this value is
     * not provided, the point defaults to the enemy goal
     */
    void updateControlParams(std::optional<Point> chip_target);

    void updateAiConfig(const TbotsProto::AiConfig& ai_config);

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    TbotsProto::AiConfig ai_config;  // AI config
    std::shared_ptr<Strategy> strategy;

    SkillGraph skill_graph_;
    std::map<RobotId, std::shared_ptr<Skill>> skill_map_;
    std::map<RobotId, std::unique_ptr<SkillFSM>> skill_fsm_map_;
};
