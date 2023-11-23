#include "software/ai/hl/stp/skill/skill_visitor.h"

#include <memory>

void function()
{
    SkillScorer skill_scorer;
    std::shared_ptr<Skill> ds = std::make_shared<DribbleSkill>();
    skill_scorer.score(*ds);
}
