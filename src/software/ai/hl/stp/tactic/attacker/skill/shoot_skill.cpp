#include "software/ai/hl/stp/tactic/attacker/skill/shoot_skill.h"

#include "software/ai/hl/stp/tactic/attacker/skill/shoot_skill_fsm.h"
#include "software/util/generic_factory/generic_factory.h"

double ShootSkill::getViability(const Robot& robot, const World& world) const
{
    if (!(*strategy_)->getBestShot(robot))
    {
        return 0;
    }
    return 1;
}

std::unique_ptr<SkillFSM> ShootSkill::getFSM() const
{
    return std::make_unique<ShootSkillFSM>(strategy_);
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
