#pragma once

#include "software/ai/hl/stp/skill/skill.h"
#include "software/ai/evaluation/scoring/skills/skill_visitor.h"
#include "software/ai/hl/stp/strategy/strategy.h"
#include "software/world/robot.h"
#include "software/world/world.h"

class FeasibilityVisitor : public SkillVisitor
{
    public:
        FeasibilityVisitor(const Robot& robot, std::shared_ptr<Strategy> strategy, const World& world);

        void visit(const KeepAwaySkill& skill) override;
        void visit(const PassSkill& skill) override;
        void visit(const ShootSkill& skill) override;

        /**
         * Returns a viability score in the range [0, 1] indicating whether the Skill
         * is feasible for the given robot to execute.
         *
         * A viability score of 0 means that the Skill is inviable and the robot
         * cannot execute the Skill.
         *
         * @param robot the robot that will execute the Skill
         * @param world the World
         *
         * @return a viability score in the range [0, 1] indicating whether the skill
         * is feasible for the given robot to execute
         */
        double getFeasibility(const Skill& skill);

    private:
        double current_feasibility_;
        Robot robot_;
        std::shared_ptr<Strategy> strategy_;
        World world_;
};
