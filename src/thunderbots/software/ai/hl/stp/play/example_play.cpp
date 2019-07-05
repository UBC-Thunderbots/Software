#include "ai/hl/stp/play/example_play.h"
#include "ai/hl/stp/tactic/shoot_goal_tactic.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "shared/constants.h"
#include "ai/hl/stp/tactic/grab_ball_tactic.h"
#include "ai/hl/stp/tactic/loose_ball_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"

const std::string ExamplePlay::name = "Example Play";

std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

bool ExamplePlay::isApplicable(const World &world) const
{
    return false;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return false;
}

void ExamplePlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Create MoveTactics that will loop forever
    auto loose_ball_tactic = std::make_shared<LooseBallTactic>(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), Angle::ofDegrees(4), std::nullopt, true);

        loose_ball_tactic->addWhitelistedAvoidArea(AvoidArea::BALL);
        loose_ball_tactic->addWhitelistedAvoidArea(AvoidArea::HALF_METER_AROUND_BALL);
    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    do
    {
        loose_ball_tactic->updateParams(world.field(), world.friendlyTeam(),
                                           world.enemyTeam(), world.ball(), std::nullopt);
        goalie_tactic->updateParams(world.ball(), world.field(), world.friendlyTeam(),
                                    world.enemyTeam());
        yield({goalie_tactic, loose_ball_tactic});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<ExamplePlay> factory;
