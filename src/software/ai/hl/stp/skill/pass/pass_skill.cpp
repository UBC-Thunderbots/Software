#include "software/ai/hl/stp/skill/pass/pass_skill.h"

#include "software/util/generic_factory/generic_factory.h"

PassSkill::PassSkill(std::shared_ptr<Strategy> strategy)
    : BaseSkill(strategy)
{
}

void PassSkill::accept(SkillVisitor& visitor)
{
    visitor.visit(*this);
}


// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, PassSkill, std::shared_ptr<Strategy>> factory;
