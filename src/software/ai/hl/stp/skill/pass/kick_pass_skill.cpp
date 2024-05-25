#include "software/ai/hl/stp/skill/pass/kick_pass_skill.h"

#include "software/util/generic_factory/generic_factory.h"

KickPassSkill::KickPassSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

void KickPassSkill::updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                                    const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.should_chip = false};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KickPassSkill, std::shared_ptr<Strategy>>
    factory;
