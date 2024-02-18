#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double KeepAwaySkill::getViability(const Robot& robot, const World& world) const
{
    constexpr double NEAR_CREASE_PENALTY_MAX = 0.25;

    // Penalize viability if we are close to the crease
    double near_crease_penalty = normalizeValueToRange(
        distance(robot.position(), world.field().enemyDefenseArea()), 0.0, 1.0,
        NEAR_CREASE_PENALTY_MAX, 0.0);

    return 1 - near_crease_penalty;
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KeepAwaySkill, std::shared_ptr<Strategy>>
    factory;
