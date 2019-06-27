#include "free_kick_tactic.h"

#include "ai/hl/stp/action/chip_action.h"
#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/evaluation/deflect_off_enemy_target.h"
#include "ai/hl/stp/evaluation/indirect_chip.h"
#include "shared/constants.h"
#include "util/logger/init.h"

namespace
{
    /**
     * Minimum opening seen by robot for a shot on goal.
     */
    const Angle MIN_SHOT_ANGLE = Angle::ofDegrees(15);

    /**
     * Scaling factor for the chip distance (ball - target)
     */
    const double CHIP_DISTANCE_SCALING_FACTOR = 0.8;

    /**
     * Max ball velocity for this tactic to be not done.
     */
    const double MAX_BALL_VELOCITY = 1.5;
}  // namespace

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
    ChipAction chip_action = ChipAction();
    Point target           = world.field().enemyGoal();

    do
    {
        // Get best shot, if any
        auto best_shot = Evaluation::calcBestShotOnEnemyGoal(
            world.field(), world.friendlyTeam(), world.enemyTeam(),
            world.ball().position());

        auto chip_and_chase_shot =
            Evaluation::findTargetPointForIndirectChipAndChase(world);

        // Check if there is a shot with a big enough window
        if (best_shot && (std::get<1>(*best_shot) > MIN_SHOT_ANGLE))
        {
            target = std::get<0>(*best_shot);
            yield(kick_action.updateStateAndGetNextIntent(
                *robot, world.ball(), world.ball().position(), target,
                BALL_MAX_SPEED_METERS_PER_SECOND));
        }
        else if (chip_and_chase_shot)
        {
            // Chip and Chase
            // TODO: possibly modify the evaluation so that it chips over the defending
            // enemies
            double chip_power = (*chip_and_chase_shot - world.ball().position()).len() *
                                CHIP_DISTANCE_SCALING_FACTOR;
            yield(chip_action.updateStateAndGetNextIntent(
                *robot, world.ball(), world.ball().position(), *chip_and_chase_shot,
                chip_power));
        }
        else
        {
            // No shot found, shoot at enemy and get deflection towards goal (hopefully)
            // note: this case rarely gets hit
            target = Evaluation::deflect_off_enemy_target(world);
            yield(kick_action.updateStateAndGetNextIntent(
                *robot, world.ball(), world.ball().position(), target,
                BALL_MAX_SPEED_METERS_PER_SECOND));
        }
    } while (world.ball().velocity().len() < MAX_BALL_VELOCITY);
}