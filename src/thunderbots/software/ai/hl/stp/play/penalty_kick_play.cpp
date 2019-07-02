#include "ai/hl/stp/play/penalty_kick_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/penalty_kick_tactic.h"
#include "ai/hl/stp/tactic/stop_tactic.h"
#include "shared/constants.h"

const std::string PenaltyKickPlay::name = "Penalty Kick Play";

std::string PenaltyKickPlay::getName() const
{
    return PenaltyKickPlay::name;
}

bool PenaltyKickPlay::isApplicable(const World &world) const
{
    if(world.gameState().isOurPenalty()) {
        return true;
    }
    else {
        return false;
    }

}

bool PenaltyKickPlay::invariantHolds(const World &world) const
{
    if(world.gameState().isOurPenalty()) {
        return true;
    }
    else {
        return false;
    }
}

void PenaltyKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    auto penalty_shot_tactic = std::make_shared<PenaltyKickTactic>(world.ball(), world.field(), world.enemyTeam().goalie(), true);

    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    do
    {
        // Assign one robot to be the penalty shooter
        penalty_shot_tactic->updateParams(world.ball(), world.enemyTeam().goalie(), world.field());

        // Move all non-shooter robots to the center of the field
        move_tactic_2->updateParams(Point(0, 0), world.field().enemyGoal().orientation(), 0 );
        move_tactic_3->updateParams(Point(0, 2*ROBOT_MAX_RADIUS_METERS), world.field().enemyGoal().orientation(), 0 );
        move_tactic_4->updateParams(Point(0, -2*ROBOT_MAX_RADIUS_METERS), world.field().enemyGoal().orientation(), 0 );
        move_tactic_5->updateParams(Point(0, 4*ROBOT_MAX_RADIUS_METERS), world.field().enemyGoal().orientation(), 0 );
        move_tactic_6->updateParams(Point(0,-4*ROBOT_MAX_RADIUS_METERS), world.field().enemyGoal().orientation(), 0 );

        // yield the Tactics this Play wants to run, in order of priority
        yield({penalty_shot_tactic, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<PenaltyKickPlay> factory;
