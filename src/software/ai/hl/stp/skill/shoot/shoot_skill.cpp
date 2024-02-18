#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double ShootSkill::getViability(const Robot& robot, const World& world) const
{
    std::optional<Shot> best_shot = (*strategy_)->getBestShot(robot);
    if (!best_shot)
    {
        return 0;
    }

    constexpr double SHOT_ANGLE_PENALTY_MAX = 0.25;

    double shot_angle_penalty = normalizeValueToRange(
        best_shot->getOpenAngle().toDegrees(), 0.0, 180.0, SHOT_ANGLE_PENALTY_MAX,
        0.0);

    return 1 - shot_angle_penalty;
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
