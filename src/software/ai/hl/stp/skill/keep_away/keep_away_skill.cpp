#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/util/generic_factory/generic_factory.h"

KeepAwaySkill::KeepAwaySkill(std::shared_ptr<Strategy> strategy) : BaseSkill(strategy) {}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KeepAwaySkill, std::shared_ptr<Strategy>>
    factory;
