#include "free_kick_tactic.h"

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/deflect_off_enemy_target.h"
#include "ai/hl/stp/evaluation/indirect_chip.h"
#include "shared/constants.h"

FreeKickTactic::FreeKickTactic(const World& world, bool loop_forever)
    : world(world), Tactic(loop_forever)
{
}

std::string FreeKickTactic::getName() const
{
    return "Free Kick Tactic";
}

void FreeKickTactic::updateParams(const World& updated_world)
{
    this->world = updated_world;
}


double FreeKickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    return dist(robot.position(), world.ball().position());
}


void FreeKickTactic::calculateNextIntent(IntentCoroutine::push_type& yield)
{
    KickAction kick_action = KickAction();
    Point target           = world.field().enemyGoal();

    do
    {
        // Get best shot, if any
        auto best_shot = Evaluation::calcBestShotOnEnemyGoal(
            world.field(), world.friendlyTeam(), world.enemyTeam(),
            world.ball().position());

        auto chip_and_chase_shot =
            Evaluation::findTargetPointForIndirectChipAndChase(world);

        if (best_shot)
        {
            target = std::get<0>(*best_shot);
        }
        else if (chip_and_chase_shot)
        {
            // Chip and Chase; TODO
        }
        else
        {
            // No shot found, shoot at enemy and get deflection towards goal (hopefully)
            target = Evaluation::deflect_off_enemy_target(world);
        }
        yield(kick_action.updateStateAndGetNextIntent(*robot, world.ball(),
                                                      world.ball().position(), target,
                                                      BALL_MAX_SPEED_METERS_PER_SECOND));

        // Temporary done condition
    } while (world.ball().velocity().len() < 2.0);
}