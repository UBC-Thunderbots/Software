#pragma once

#include "software/ai/evaluation/scoring/skills/skill_graph.h"
#include "software/ai/hl/stp/tactic/tactic.h"

class AttackerTactic : public Tactic
{
   public:
    explicit AttackerTactic(std::shared_ptr<Strategy> strategy);

    AttackerTactic() = delete;

    void accept(TacticVisitor& visitor) const override;

    inline bool done() const override
    {
        return false;
    }

    std::string getFSMState() const override;

    void setLastExecutionRobot(std::optional<RobotId> last_execution_robot) override;

    // TODO: Remove these
    void updateControlParams(const Pass& best_pass_so_far, bool pass_committed) {}
    void updateControlParams(std::optional<Point> chip_target) {}

   private:
    std::shared_ptr<Strategy> strategy;

    bool last_execution_robot_changed_;

    SkillGraph skill_graph_;
    std::unordered_map<RobotId, std::shared_ptr<Skill>> skill_map_;
    std::unordered_map<RobotId, std::unique_ptr<SkillFSM>> skill_fsm_map_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};
