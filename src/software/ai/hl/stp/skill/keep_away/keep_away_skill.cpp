#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/ai/evaluation/scoring/skills/skill_visitor.h"
#include "software/util/generic_factory/generic_factory.h"

KeepAwaySkill::KeepAwaySkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

void KeepAwaySkill::accept(SkillVisitor &skill_visitor)
{
    skill_visitor.visit(*this);
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KeepAwaySkill, std::shared_ptr<Strategy>> factory;
