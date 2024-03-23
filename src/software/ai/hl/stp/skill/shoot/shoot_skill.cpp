#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"

#include "software/util/generic_factory/generic_factory.h"

ShootSkill::ShootSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

void ShootSkill::accept(SkillVisitor& visitor)
{
    visitor.visit(*this);
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
