#include "software/ai/hl/stp/skill/shoot/shoot_skill.h"

#include "software/util/generic_factory/generic_factory.h"

double ShootSkill::getViability(const Robot& robot, const World& world) const
{
    std::optional<Shot> best_shot = (*strategy_)->getBestShot(robot);
    if (!best_shot)
    {
        return 0;
    }

    double shot_angle_viability =
        normalizeValueToRange(best_shot->getOpenAngle().toDegrees(), 0.0, 60.0, 0.0, 1.0);
    return shot_angle_viability;
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, ShootSkill, std::shared_ptr<Strategy>> factory;
