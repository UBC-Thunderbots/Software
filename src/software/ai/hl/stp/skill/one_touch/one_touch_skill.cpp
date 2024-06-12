#include "software/ai/hl/stp/skill/one_touch/one_touch_skill.h"

#include "software/util/generic_factory/generic_factory.h"

OneTouchSkill::OneTouchSkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, OneTouchSkill, std::shared_ptr<Strategy>> factory;
