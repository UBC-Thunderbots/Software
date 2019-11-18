#include "software/ai/hl/stp/play/penalty_kick_enemy_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/play/play_factory.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"

const std::string PenaltyKickEnemyPlay::name = "Penalty Kick Enemy Play";

std::string PenaltyKickEnemyPlay::getName() const
{
    return PenaltyKickEnemyPlay::name;
}

bool PenaltyKickEnemyPlay::isApplicable(const World &world) const
{
    return world.gameState().isTheirPenalty();
}

bool PenaltyKickEnemyPlay::invariantHolds(const World &world) const
{
    return world.gameState().isTheirPenalty() && !world.gameState().isStopped() &&
           !world.gameState().isHalted();
}

void PenaltyKickEnemyPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    do
    {
        // goalie
        goalie_tactic->updateWorldParams(world.ball(), world.field(),
                                         world.friendlyTeam(), world.enemyTeam());

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // Move all non-shooter robots to the center of the field
        move_tactic_2->updateControlParams(
            Point(0, 0), world.field().enemyGoal().toVector().orientation(), 0);
        move_tactic_3->updateControlParams(
            Point(0, 4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoal().toVector().orientation(), 0);
        move_tactic_4->updateControlParams(
            Point(0, -4 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoal().toVector().orientation(), 0);
        move_tactic_5->updateControlParams(
            Point(0, 8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoal().toVector().orientation(), 0);
        move_tactic_6->updateControlParams(
            Point(0, -8 * ROBOT_MAX_RADIUS_METERS),
            world.field().enemyGoal().toVector().orientation(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        yield({goalie_tactic, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<PenaltyKickEnemyPlay> factory;
