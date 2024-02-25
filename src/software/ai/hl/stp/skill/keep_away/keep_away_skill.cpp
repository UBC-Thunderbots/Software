#include "software/ai/hl/stp/skill/keep_away/keep_away_skill.h"

#include "software/ai/evaluation/keep_away.h"
#include "software/util/generic_factory/generic_factory.h"

double KeepAwaySkill::getViability(const Robot& robot, const World& world) const
{
    if (world.field().pointInFriendlyHalf(robot.position()) ||
        contains(world.field().enemyDefenseArea().expand(0.1), robot.position()))
    {
        return 0;
    }

    if (!shouldKeepAway(robot, world.enemyTeam(),
                        strategy_->getAiConfig()
                            .attacker_tactic_config()
                            .enemy_about_to_steal_ball_radius()))
    {
        return 0;
    }

    return 1;
}

// Register this Skill in the GenericFactory
static TGenericFactory<std::string, Skill, KeepAwaySkill, std::shared_ptr<Strategy>>
    factory;
