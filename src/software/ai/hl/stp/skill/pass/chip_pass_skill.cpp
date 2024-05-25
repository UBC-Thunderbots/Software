#include "software/ai/hl/stp/skill/pass/chip_pass_skill.h"

#include "software/util/generic_factory/generic_factory.h"

ChipPassSkill::ChipPassSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

void ChipPassSkill::updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                                    const SetPrimitiveCallback& set_primitive)
{
    control_params_ = {.should_chip = true};
    BaseSkill::updatePrimitive(robot, world_ptr, set_primitive);
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ChipPassSkill, std::shared_ptr<Strategy>>
    factory;
