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

    // TODO: Remove these
    void updateControlParams(const Pass& best_pass_so_far, bool pass_committed) {}
    void updateControlParams(std::optional<Point> chip_target) {}

   private:
    std::shared_ptr<Strategy> strategy;

    SkillGraph skill_graph_;
    std::shared_ptr<Skill> current_skill_;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;
};
